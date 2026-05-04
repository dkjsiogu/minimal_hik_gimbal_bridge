#include "bridge/hik_camera.hpp"

#include <MvCameraControl.h>
#include <PixelType.h>

#include <algorithm>
#include <cstring>
#include <sstream>
#include <stdexcept>

namespace bridge
{

HikCamera::HikCamera() = default;

HikCamera::~HikCamera()
{
  close();
}

bool HikCamera::is_open() const
{
  return handle_ != nullptr;
}

const std::string & HikCamera::last_error() const
{
  return last_error_;
}

double HikCamera::exposure_ms() const
{
  return exposure_ms_;
}

double HikCamera::gain() const
{
  return gain_;
}

void HikCamera::open_first(double exposure_ms, double gain)
{
  close();

  MV_CC_DEVICE_INFO_LIST device_list;
  std::memset(&device_list, 0, sizeof(device_list));

  const auto ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    throw std::runtime_error(
      "MV_CC_EnumDevices(MV_USB_DEVICE) 失败: 0x" + hex_code(ret) +
      "；请确认运行时 LD_LIBRARY_PATH 包含完整 MVS 运行时，例如 /opt/MVS/lib/64");
  }

  if (device_list.nDeviceNum == 0) {
    throw std::runtime_error("未找到海康相机");
  }

  check(MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]), "MV_CC_CreateHandle");
  try {
    check(MV_CC_OpenDevice(handle_), "MV_CC_OpenDevice");
    apply_settings_or_throw(exposure_ms, gain);
    check(MV_CC_StartGrabbing(handle_), "MV_CC_StartGrabbing");
    grabbing_ = true;
    last_error_.clear();
  } catch (...) {
    close();
    throw;
  }
}

bool HikCamera::apply_settings(double exposure_ms, double gain, std::string * error)
{
  if (!is_open()) {
    exposure_ms_ = std::max(0.1, exposure_ms);
    gain_ = std::max(0.0, gain);
    return true;
  }

  try {
    apply_settings_or_throw(exposure_ms, gain);
    last_error_.clear();
    return true;
  } catch (const std::exception & current_error) {
    last_error_ = current_error.what();
    if (error != nullptr) {
      *error = last_error_;
    }
    return false;
  }
}

CameraGrabResult HikCamera::grab(FrameInfo & frame, unsigned int timeout_ms, bool copy_rgb24)
{
  if (!is_open()) {
    last_error_ = "海康相机尚未打开";
    return CameraGrabResult::DeviceError;
  }

  MV_FRAME_OUT raw;
  std::memset(&raw, 0, sizeof(raw));

  const auto ret = MV_CC_GetImageBuffer(handle_, &raw, timeout_ms);
  if (ret == MV_E_NODATA || ret == MV_E_GC_TIMEOUT) {
    return CameraGrabResult::Timeout;
  }
  if (ret != MV_OK) {
    last_error_ = "MV_CC_GetImageBuffer 失败: 0x" + hex_code(static_cast<unsigned int>(ret));
    close();
    return CameraGrabResult::DeviceError;
  }

  try {
    frame.width = static_cast<uint16_t>(raw.stFrameInfo.nWidth);
    frame.height = static_cast<uint16_t>(raw.stFrameInfo.nHeight);
    frame.bytes = raw.stFrameInfo.nFrameLen;
    frame.sequence = raw.stFrameInfo.nFrameNum;
    frame.timestamp = Clock::now();

    if (copy_rgb24) {
      frame.rgb24.resize(static_cast<std::size_t>(frame.width) * frame.height * 3U);

      MV_CC_PIXEL_CONVERT_PARAM_EX convert_param;
      std::memset(&convert_param, 0, sizeof(convert_param));
      convert_param.nWidth = raw.stFrameInfo.nWidth;
      convert_param.nHeight = raw.stFrameInfo.nHeight;
      convert_param.enSrcPixelType = raw.stFrameInfo.enPixelType;
      convert_param.pSrcData = raw.pBufAddr;
      convert_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
      convert_param.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
      convert_param.pDstBuffer = frame.rgb24.data();
      convert_param.nDstBufferSize = static_cast<unsigned int>(frame.rgb24.size());

      const auto convert_ret = MV_CC_ConvertPixelTypeEx(handle_, &convert_param);
      const auto free_ret = MV_CC_FreeImageBuffer(handle_, &raw);
      if (convert_ret != MV_OK) {
        throw std::runtime_error("MV_CC_ConvertPixelTypeEx 失败: 0x" + hex_code(static_cast<unsigned int>(convert_ret)));
      }
      check(free_ret, "MV_CC_FreeImageBuffer");
      frame.rgb24.resize(convert_param.nDstLen);
      last_error_.clear();
      return CameraGrabResult::Success;
    }

    check(MV_CC_FreeImageBuffer(handle_, &raw), "MV_CC_FreeImageBuffer");
    last_error_.clear();
    return CameraGrabResult::Success;
  } catch (const std::exception & current_error) {
    last_error_ = current_error.what();
    close();
    return CameraGrabResult::DeviceError;
  }
}

void HikCamera::close()
{
  if (handle_ == nullptr) {
    return;
  }

  if (grabbing_) {
    MV_CC_StopGrabbing(handle_);
    grabbing_ = false;
  }
  MV_CC_CloseDevice(handle_);
  MV_CC_DestroyHandle(handle_);
  handle_ = nullptr;
}

void HikCamera::apply_settings_or_throw(double exposure_ms, double gain)
{
  double exposure_us = std::max(0.1, exposure_ms) * 1000.0;
  double target_gain = std::max(0.0, gain);

  check(MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF), "Set ExposureAuto");
  check(MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF), "Set GainAuto");
  exposure_us = clamp_float_node("ExposureTime", exposure_us);
  target_gain = clamp_float_node("Gain", target_gain);
  check(MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_us), "Set ExposureTime");
  check(MV_CC_SetFloatValue(handle_, "Gain", target_gain), "Set Gain");

  exposure_ms_ = exposure_us / 1000.0;
  gain_ = target_gain;
}

double HikCamera::clamp_float_node(const char * node_name, double value) const
{
  MVCC_FLOATVALUE range;
  std::memset(&range, 0, sizeof(range));
  const auto ret = MV_CC_GetFloatValue(handle_, node_name, &range);
  if (ret != MV_OK || range.fMax < range.fMin) {
    return value;
  }

  return std::clamp(value, static_cast<double>(range.fMin), static_cast<double>(range.fMax));
}

std::string HikCamera::hex_code(unsigned int value)
{
  std::ostringstream oss;
  oss << std::hex << std::uppercase << value;
  return oss.str();
}

void HikCamera::check(int code, const char * action)
{
  if (code != MV_OK) {
    throw std::runtime_error(std::string(action) + " 失败: 0x" + hex_code(static_cast<unsigned int>(code)));
  }
}

}  // namespace bridge
