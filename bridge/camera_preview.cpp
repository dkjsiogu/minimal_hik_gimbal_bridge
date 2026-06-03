#include "bridge/camera_preview.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <vector>

namespace bridge
{

namespace
{

constexpr int kPreviewMaxWidth = 1280;
constexpr int kPreviewMaxHeight = 720;

bool is_identity_rotation_matrix(const std::array<double, 4> & matrix)
{
  return classify_right_angle_rotation(matrix) == RightAngleRotation::kIdentity;
}

cv::Matx23d build_rotation_affine(const cv::Size & size, const std::array<double, 4> & matrix)
{
  const double cx = (static_cast<double>(size.width) - 1.0) * 0.5;
  const double cy = (static_cast<double>(size.height) - 1.0) * 0.5;
  return cv::Matx23d(
    matrix[0], matrix[1], (1.0 - matrix[0]) * cx - matrix[1] * cy,
    matrix[2], matrix[3], -matrix[2] * cx + (1.0 - matrix[3]) * cy);
}

cv::Matx23d invert_affine(const cv::Matx23d & affine)
{
  const cv::Matx22d linear(affine(0, 0), affine(0, 1), affine(1, 0), affine(1, 1));
  const cv::Matx22d inverse_linear = linear.inv();
  const cv::Vec2d translation(affine(0, 2), affine(1, 2));
  const cv::Vec2d inverse_translation = -(inverse_linear * translation);
  return cv::Matx23d(
    inverse_linear(0, 0), inverse_linear(0, 1), inverse_translation(0),
    inverse_linear(1, 0), inverse_linear(1, 1), inverse_translation(1));
}

cv::Point2d transform_point(const cv::Matx23d & affine, const cv::Point2d & point)
{
  return cv::Point2d(
    affine(0, 0) * point.x + affine(0, 1) * point.y + affine(0, 2),
    affine(1, 0) * point.x + affine(1, 1) * point.y + affine(1, 2));
}

cv::Point scale_point_to_display(const cv::Point2d & source_point, const cv::Size & source_size, const cv::Size & display_size)
{
  if (source_size.width <= 0 || source_size.height <= 0 ||
      display_size.width <= 0 || display_size.height <= 0) {
    return {};
  }

  const int display_x = std::clamp(
    static_cast<int>(std::lround(source_point.x * static_cast<double>(display_size.width) /
      static_cast<double>(source_size.width))),
    0,
    display_size.width - 1);
  const int display_y = std::clamp(
    static_cast<int>(std::lround(source_point.y * static_cast<double>(display_size.height) /
      static_cast<double>(source_size.height))),
    0,
    display_size.height - 1);
  return cv::Point(display_x, display_y);
}

std::vector<cv::Point> inverse_mapped_crop_polygon(
  const cv::Rect & rotated_rect,
  const cv::Size & source_size,
  const cv::Size & display_size,
  const cv::Matx23d & inverse_affine)
{
  if (rotated_rect.width <= 0 || rotated_rect.height <= 0) {
    return {};
  }

  const std::array<cv::Point2d, 4> rotated_corners = {
    cv::Point2d(rotated_rect.x, rotated_rect.y),
    cv::Point2d(rotated_rect.x + rotated_rect.width, rotated_rect.y),
    cv::Point2d(rotated_rect.x + rotated_rect.width, rotated_rect.y + rotated_rect.height),
    cv::Point2d(rotated_rect.x, rotated_rect.y + rotated_rect.height)};

  std::vector<cv::Point> polygon;
  polygon.reserve(rotated_corners.size());
  for (const auto & corner : rotated_corners) {
    polygon.push_back(scale_point_to_display(transform_point(inverse_affine, corner), source_size, display_size));
  }
  return polygon;
}

cv::Size preview_display_size(const cv::Size & source_size)
{
  if (source_size.width <= 0 || source_size.height <= 0) {
    return {};
  }

  const double scale = std::min(
    1.0,
    std::min(
      static_cast<double>(kPreviewMaxWidth) / static_cast<double>(source_size.width),
      static_cast<double>(kPreviewMaxHeight) / static_cast<double>(source_size.height)));
  return cv::Size(
    std::max(1, static_cast<int>(std::lround(static_cast<double>(source_size.width) * scale))),
    std::max(1, static_cast<int>(std::lround(static_cast<double>(source_size.height) * scale))));
}


cv::Point2d map_source_point_to_rotated_preview(
  const cv::Point2d & source_point,
  const cv::Size & source_size,
  const std::array<double, 4> & matrix)
{
  if (classify_right_angle_rotation(matrix) != RightAngleRotation::kUnsupported) {
    return map_source_point_to_rotated_frame(source_point, source_size, matrix);
  }
  return transform_point(build_rotation_affine(source_size, matrix), source_point);
}

cv::Point2d map_rotated_point_to_source_preview(
  const cv::Point2d & rotated_point,
  const cv::Size & source_size,
  const std::array<double, 4> & matrix)
{
  if (classify_right_angle_rotation(matrix) != RightAngleRotation::kUnsupported) {
    return map_rotated_point_to_source_frame(rotated_point, source_size, matrix);
  }
  return transform_point(invert_affine(build_rotation_affine(source_size, matrix)), rotated_point);
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

  options_ = &options;
  ensure_window();

  if (frame != nullptr && !frame->rgb24.empty() && frame->width > 0 && frame->height > 0) {
    cv::Mat input_rgb(
      static_cast<int>(frame->height),
      static_cast<int>(frame->width),
      CV_8UC3,
      const_cast<uint8_t *>(frame->rgb24.data()));
    cv::Mat input_bgr;
    cv::cvtColor(input_rgb, input_bgr, cv::COLOR_RGB2BGR);

    source_size_ = input_bgr.size();

    rotated_size_ = rotated_frame_size(source_size_, options.rotation_matrix);
    const auto crop_region = compute_square_crop_region(
      rotated_size_, options.crop_size, options.crop_center_x, options.crop_center_y);
    options.crop_center_x = crop_region.center_x;
    options.crop_center_y = crop_region.center_y;

    display_size_ = preview_display_size(source_size_);
    cv::Mat display = input_bgr;
    if (display_size_ != source_size_) {
      cv::resize(input_bgr, display, display_size_, 0, 0, cv::INTER_LINEAR);
    } else {
      display = input_bgr.clone();
    }

    std::vector<cv::Point> roi_polygon;
    roi_polygon.reserve(4);
    const std::array<cv::Point2d, 4> rotated_corners = {
      cv::Point2d(crop_region.rect.x, crop_region.rect.y),
      cv::Point2d(crop_region.rect.x + crop_region.rect.width, crop_region.rect.y),
      cv::Point2d(crop_region.rect.x + crop_region.rect.width, crop_region.rect.y + crop_region.rect.height),
      cv::Point2d(crop_region.rect.x, crop_region.rect.y + crop_region.rect.height)};
    for (const auto & corner : rotated_corners) {
      roi_polygon.push_back(scale_point_to_display(
        map_rotated_point_to_source_preview(corner, source_size_, options.rotation_matrix),
        source_size_,
        display.size()));
    }
    if (roi_polygon.size() == 4) {
      cv::polylines(display, roi_polygon, true, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }

    const cv::Point2d rotated_center(
      crop_region.rect.x + static_cast<double>(crop_region.rect.width) * 0.5,
      crop_region.rect.y + static_cast<double>(crop_region.rect.height) * 0.5);
    const auto display_center = scale_point_to_display(
      map_rotated_point_to_source_preview(rotated_center, source_size_, options.rotation_matrix),
      source_size_,
      display.size());
    cv::circle(display, display_center, 4, cv::Scalar(0, 255, 255), cv::FILLED, cv::LINE_AA);
    cv::putText(
      display,
      "Drag on raw preview to choose transmitted area | S save",
      cv::Point(16, std::max(24, display.rows - 18)),
      cv::FONT_HERSHEY_SIMPLEX,
      0.55,
      cv::Scalar(0, 255, 255),
      1,
      cv::LINE_AA);
    cv::imshow(kWindowName, display);
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
                << " gain=" << options.gain
                << " crop_center=(" << options.crop_center_x << ',' << options.crop_center_y << ')' << std::endl;
    } else if (!error.empty()) {
      std::cerr << "[preview] 保存配置失败: " << error << std::endl;
    }
  }
}

void CameraPreviewWindow::on_mouse(int event, int x, int y, int flags, void * userdata)
{
  if (userdata == nullptr) {
    return;
  }
  static_cast<CameraPreviewWindow *>(userdata)->handle_mouse(event, x, y, flags);
}

void CameraPreviewWindow::handle_mouse(int event, int x, int y, int flags)
{
  if (options_ == nullptr || display_size_.width <= 0 || display_size_.height <= 0) {
    return;
  }

  const bool inside = x >= 0 && y >= 0 && x < display_size_.width && y < display_size_.height;
  if (event == cv::EVENT_LBUTTONDOWN && inside) {
    dragging_roi_ = true;
    update_crop_center_from_display_point(x, y);
    return;
  }

  if (event == cv::EVENT_MOUSEMOVE && dragging_roi_ && (flags & cv::EVENT_FLAG_LBUTTON) != 0) {
    update_crop_center_from_display_point(x, y);
    return;
  }

  if (event == cv::EVENT_LBUTTONUP) {
    if (dragging_roi_ && inside) {
      update_crop_center_from_display_point(x, y);
    }
    dragging_roi_ = false;
  }
}

void CameraPreviewWindow::update_crop_center_from_display_point(int x, int y)
{
  if (options_ == nullptr || source_size_.width <= 0 || source_size_.height <= 0 ||
      rotated_size_.width <= 0 || rotated_size_.height <= 0 ||
      display_size_.width <= 0 || display_size_.height <= 0) {
    return;
  }

  const double source_x = std::clamp(
    ((static_cast<double>(x) + 0.5) / static_cast<double>(display_size_.width)) *
      static_cast<double>(source_size_.width) - 0.5,
    0.0,
    static_cast<double>(source_size_.width - 1));
  const double source_y = std::clamp(
    ((static_cast<double>(y) + 0.5) / static_cast<double>(display_size_.height)) *
      static_cast<double>(source_size_.height) - 0.5,
    0.0,
    static_cast<double>(source_size_.height - 1));

  const auto rotated_point = map_source_point_to_rotated_preview(
    cv::Point2d(source_x, source_y), source_size_, options_->rotation_matrix);
  const double normalized_x = rotated_size_.width > 1
    ? std::clamp(rotated_point.x / static_cast<double>(rotated_size_.width - 1), 0.0, 1.0)
    : 0.5;
  const double normalized_y = rotated_size_.height > 1
    ? std::clamp(rotated_point.y / static_cast<double>(rotated_size_.height - 1), 0.0, 1.0)
    : 0.5;
  const auto crop_region = compute_square_crop_region(
    rotated_size_, options_->crop_size, normalized_x, normalized_y);
  options_->crop_center_x = crop_region.center_x;
  options_->crop_center_y = crop_region.center_y;
}

void CameraPreviewWindow::ensure_window()
{
  if (window_created_) {
    return;
  }

  cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
  cv::createTrackbar("Exposure x0.1ms", kWindowName, nullptr, kMaxExposureTrackbar);
  cv::createTrackbar("Gain x0.1", kWindowName, nullptr, kMaxGainTrackbar);
  cv::setTrackbarPos("Exposure x0.1ms", kWindowName, exposure_tenths_ms_);
  cv::setTrackbarPos("Gain x0.1", kWindowName, gain_tenths_);
  cv::setMouseCallback(kWindowName, &CameraPreviewWindow::on_mouse, this);
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
