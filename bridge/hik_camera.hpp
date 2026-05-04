#pragma once

#include "bridge/frame.hpp"

#include <string>

namespace bridge
{

enum class CameraGrabResult
{
  Success,
  Timeout,
  DeviceError,
};

class HikCamera
{
public:
  HikCamera();
  ~HikCamera();

  bool is_open() const;
  const std::string & last_error() const;
  double exposure_ms() const;
  double gain() const;
  void open_first(double exposure_ms, double gain);
  bool apply_settings(double exposure_ms, double gain, std::string * error = nullptr);
  CameraGrabResult grab(FrameInfo & frame, unsigned int timeout_ms, bool copy_rgb24);
  void close();

private:
  void apply_settings_or_throw(double exposure_ms, double gain);
  double clamp_float_node(const char * node_name, double value) const;
  static std::string hex_code(unsigned int value);
  static void check(int code, const char * action);

  void * handle_ = nullptr;
  bool grabbing_ = false;
  double exposure_ms_ = 10.0;
  double gain_ = 12.0;
  std::string last_error_;
};

}  // namespace bridge
