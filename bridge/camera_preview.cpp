#include "bridge/camera_preview.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

namespace bridge
{

namespace
{

cv::Mat apply_rotation_matrix_bgr(const cv::Mat & input_bgr, const std::array<double, 4> & matrix)
{
  if (input_bgr.empty()) {
    return {};
  }

  const bool is_identity =
    std::fabs(matrix[0] - 1.0) < 1e-9 &&
    std::fabs(matrix[1]) < 1e-9 &&
    std::fabs(matrix[2]) < 1e-9 &&
    std::fabs(matrix[3] - 1.0) < 1e-9;
  if (is_identity) {
    return input_bgr;
  }

  const double cx = (static_cast<double>(input_bgr.cols) - 1.0) * 0.5;
  const double cy = (static_cast<double>(input_bgr.rows) - 1.0) * 0.5;
  cv::Mat affine = (cv::Mat_<double>(2, 3) <<
    matrix[0], matrix[1], (1.0 - matrix[0]) * cx - matrix[1] * cy,
    matrix[2], matrix[3], -matrix[2] * cx + (1.0 - matrix[3]) * cy);

  cv::Mat rotated;
  cv::warpAffine(
    input_bgr,
    rotated,
    affine,
    input_bgr.size(),
    cv::INTER_LINEAR,
    cv::BORDER_CONSTANT,
    cv::Scalar::all(0));
  return rotated;
}

}  // namespace

CameraPreviewWindow::CameraPreviewWindow(const Options & options)
: enabled_(options.preview),
  exposure_tenths_ms_(to_exposure_slider(options.exposure_ms)),
  gain_tenths_(to_gain_slider(options.gain))
{
}

CameraPreviewWindow::~CameraPreviewWindow()
{
  if (window_created_) {
    cv::destroyWindow(kWindowName);
  }
}

bool CameraPreviewWindow::enabled() const
{
  return enabled_;
}

void CameraPreviewWindow::pump(Options & options, HikCamera & camera, const FrameInfo * frame)
{
  if (!enabled_) {
    return;
  }

  ensure_window();

  if (frame != nullptr && !frame->rgb24.empty() && frame->width > 0 && frame->height > 0) {
    cv::Mat input_rgb(
      static_cast<int>(frame->height),
      static_cast<int>(frame->width),
      CV_8UC3,
      const_cast<uint8_t *>(frame->rgb24.data()));
    cv::Mat input_bgr;
    cv::cvtColor(input_rgb, input_bgr, cv::COLOR_RGB2BGR);
    cv::imshow(kWindowName, apply_rotation_matrix_bgr(input_bgr, options.rotation_matrix));
  }

  const int key = cv::waitKey(1);
  exposure_tenths_ms_ = cv::getTrackbarPos("Exposure x0.1ms", kWindowName);
  gain_tenths_ = cv::getTrackbarPos("Gain x0.1", kWindowName);

  const double desired_exposure_ms = std::max(0.1, exposure_tenths_ms_ / static_cast<double>(kExposureScale));
  const double desired_gain = std::max(0.0, gain_tenths_ / static_cast<double>(kGainScale));
  const bool exposure_changed = std::fabs(desired_exposure_ms - options.exposure_ms) >= 0.05;
  const bool gain_changed = std::fabs(desired_gain - options.gain) >= 0.05;
  if (exposure_changed || gain_changed) {
    std::string error;
    if (camera.apply_settings(desired_exposure_ms, desired_gain, &error)) {
      options.exposure_ms = camera.exposure_ms();
      options.gain = camera.gain();

      const int applied_exposure_slider = to_exposure_slider(options.exposure_ms);
      const int applied_gain_slider = to_gain_slider(options.gain);
      if (applied_exposure_slider != exposure_tenths_ms_) {
        exposure_tenths_ms_ = applied_exposure_slider;
        cv::setTrackbarPos("Exposure x0.1ms", kWindowName, exposure_tenths_ms_);
      }
      if (applied_gain_slider != gain_tenths_) {
        gain_tenths_ = applied_gain_slider;
        cv::setTrackbarPos("Gain x0.1", kWindowName, gain_tenths_);
      }
    } else if (!error.empty()) {
      const auto now = Clock::now();
      if (should_log_at_interval(now, last_error_log_, kReconnectLogInterval)) {
        std::cerr << "[preview] 相机参数应用失败，保留当前连接: " << error << std::endl;
      }
    }
  }

  if (key == 's' || key == 'S') {
    std::string error;
    if (save_config(options, &error)) {
      std::cout << "[preview] 已保存配置到 " << options.config_path
                << " exposure_ms=" << options.exposure_ms
                << " gain=" << options.gain << std::endl;
    } else if (!error.empty()) {
      std::cerr << "[preview] 保存配置失败: " << error << std::endl;
    }
  }
}

void CameraPreviewWindow::ensure_window()
{
  if (window_created_) {
    return;
  }

  cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
  cv::resizeWindow(kWindowName, 1280, 720);
  cv::createTrackbar("Exposure x0.1ms", kWindowName, nullptr, kMaxExposureTrackbar);
  cv::createTrackbar("Gain x0.1", kWindowName, nullptr, kMaxGainTrackbar);
  cv::setTrackbarPos("Exposure x0.1ms", kWindowName, exposure_tenths_ms_);
  cv::setTrackbarPos("Gain x0.1", kWindowName, gain_tenths_);
  window_created_ = true;
}

int CameraPreviewWindow::to_exposure_slider(double value)
{
  return std::clamp(
    static_cast<int>(std::lround(std::max(0.1, value) * kExposureScale)),
    1,
    kMaxExposureTrackbar);
}

int CameraPreviewWindow::to_gain_slider(double value)
{
  return std::clamp(
    static_cast<int>(std::lround(std::max(0.0, value) * kGainScale)),
    0,
    kMaxGainTrackbar);
}

}  // namespace bridge
