#pragma once

#include "bridge/frame.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace bridge
{

enum class CameraGrabResult
{
  Success,
  Timeout,
  DeviceError,
};

struct HikCameraDeviceInfo
{
  std::size_t index = 0;
  std::string serial_number;
  std::string model_name;
  std::string user_defined_name;
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
  void open_first(double exposure_ms, double gain, const std::string & serial_number = "");
  bool apply_settings(double exposure_ms, double gain, std::string * error = nullptr);
  CameraGrabResult grab(FrameInfo & frame, unsigned int timeout_ms, bool copy_rgb24);
  void close();
  static std::vector<HikCameraDeviceInfo> list_devices();

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
