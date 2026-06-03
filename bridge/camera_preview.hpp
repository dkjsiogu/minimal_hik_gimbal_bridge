#pragma once

#include "bridge/common.hpp"
#include "bridge/crop_region.hpp"
#include "bridge/frame.hpp"
#include "bridge/hik_camera.hpp"
#include "bridge/options.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace bridge
{

class CameraPreviewWindow
{
public:
  explicit CameraPreviewWindow(const Options & options);
  ~CameraPreviewWindow();

  bool enabled() const;
  void pump(Options & options, HikCamera & camera, const FrameInfo * frame);

private:
  static void on_mouse(int event, int x, int y, int flags, void * userdata);
  void handle_mouse(int event, int x, int y, int flags);
  void update_crop_center_from_display_point(int x, int y);
  void ensure_window();
  static int to_exposure_slider(double value);
  static int to_gain_slider(double value);

  static constexpr const char * kWindowName = "Hik Raw Preview";
  static constexpr int kExposureScale = 10;
  static constexpr int kGainScale = 10;
  static constexpr int kMaxExposureTrackbar = 5000;
  static constexpr int kMaxGainTrackbar = 480;

  bool enabled_ = false;
  bool window_created_ = false;
  bool dragging_roi_ = false;
  int exposure_tenths_ms_ = 100;
  int gain_tenths_ = 120;
  Options * options_ = nullptr;
  cv::Size source_size_{};
  cv::Size rotated_size_{};
  cv::Size display_size_{};
  Clock::time_point last_error_log_{};
};

}  // namespace bridge
