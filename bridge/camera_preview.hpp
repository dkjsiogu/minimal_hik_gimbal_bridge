#pragma once

#include "bridge/common.hpp"
#include "bridge/frame.hpp"
#include "bridge/hik_camera.hpp"
#include "bridge/options.hpp"

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
  int exposure_tenths_ms_ = 100;
  int gain_tenths_ = 120;
  Clock::time_point last_error_log_{};
};

}  // namespace bridge
