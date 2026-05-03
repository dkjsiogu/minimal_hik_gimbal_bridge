#include "protocol.hpp"

#include "serial/serial.h"

#include <MvCameraControl.h>
#include <PixelType.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cerrno>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

namespace
{
using Clock = std::chrono::steady_clock;

constexpr auto kGimbalOnlineTimeout = std::chrono::milliseconds(1000);

std::atomic<bool> g_running = true;

void on_signal(int)
{
  g_running = false;
}

struct Options
{
  std::string serial_port = "";
  double exposure_ms = 4.0;
  double gain = 8.0;
  int send_interval_ms = 20;
  uint8_t mode = 0;
  uint16_t sender_id = 0;
  uint16_t receiver_id = 0;
  uint16_t referee_cmd_id = 0x0310;
  std::string payload_text = "HIK+GIMBAL OK";
  bool video_0310 = true;
  std::string ffmpeg_path = "ffmpeg";
  int video_size = 300;
  int video_fps = 30;
  int video_bitrate_kbps = 88;
  int video_gop = 120;
  int crop_size = 0;
  bool static_simplify = true;
  int motion_threshold = 14;
  int motion_erode_px = 2;
  int motion_dilate_px = 6;
  int motion_trail_frames = 90;
  double trail_disable_motion_ratio = 0.30;
  double bg_update_alpha = 0.01;
  double bg_blur_sigma = 1.2;
  int center_clear_size = 100;
  bool force_monochrome = false;
  std::string viewer_ip = "192.168.1.50";
  int viewer_port = 3335;
  std::string video_serial = "/dev/ttyUSB0";
  uint32_t video_serial_baud = 921600;
};

uint16_t map_legacy_relay_target(uint32_t value)
{
  switch (value) {
    case 1:
      return static_cast<uint16_t>(bridge::protocol::RelayCommandId::SmallMap0308);
    case 2:
      return static_cast<uint16_t>(bridge::protocol::RelayCommandId::CustomController0309);
    case 3:
      return static_cast<uint16_t>(bridge::protocol::RelayCommandId::CustomClient0310);
    default:
      throw std::runtime_error("--relay-target 只支持 1=0x0308, 2=0x0309, 3=0x0310");
  }
}

void print_help()
{
  std::cout
    << "minimal_hik_gimbal_bridge\n"
    << "  --serial <path>          串口路径，默认 /dev/gimbal\n"
    << "  --exposure-ms <value>    海康曝光时间，默认 4.0 ms\n"
    << "  --gain <value>           海康增益，默认 8.0\n"
    << "  --send-interval-ms <n>   串口发送周期，默认 20 ms，对应 0x0310 50Hz\n"
    << "  --mode <0|1|2>           输出 mode，默认 0\n"
    << "  --sender-id <n>          裁判系统 sender_id，默认 0\n"
    << "  --receiver-id <n>        裁判系统 receiver_id，默认 0\n"
    << "  --referee-cmd <hex>      目标裁判命令，默认 0x0310\n"
    << "  --relay-target <1..3>    兼容旧参数，映射到 0x0308/0x0309/0x0310\n"
    << "  --payload-text <text>    自定义 payload 前缀，最长 30 字节\n"
    << "  --0310-telemetry         0310 走 telemetry 调试包，而不是视频分片\n"
    << "  --ffmpeg <path>          H264 编码器路径，默认 ffmpeg\n"
    << "  --video-size <n>         0310 视频输出边长，默认 300\n"
    << "  --video-fps <n>          0310 视频编码帧率，默认 30\n"
    << "  --video-bitrate-kbps <n> 0310 视频目标码率，默认 88 kbit/s\n"
    << "  --video-gop <n>          H264 GOP，默认 120\n"
    << "  --crop-size <n>          预处理中心裁剪边长，0 表示自动取最小边\n"
    << "  --no-static-simplify     关闭静态区域简化和拖影预处理\n"
    << "  --motion-threshold <n>   运动检测阈值，默认 14\n"
    << "  --motion-erode-px <n>    运动掩码腐蚀像素，默认 2\n"
    << "  --motion-dilate-px <n>   运动掩码膨胀像素，默认 6\n"
    << "  --motion-trail-frames <n>拖影历史帧数，默认 90\n"
    << "  --trail-disable-motion-ratio <f> 全局运动占比超阈值时禁用拖影，默认 0.30\n"
    << "  --bg-update-alpha <f>    背景更新 alpha，默认 0.01\n"
    << "  --bg-blur-sigma <f>      静态区域模糊 sigma，默认 1.2\n"
    << "  --center-clear-size <n>  中心保护区边长，默认 100\n"
    << "  --force-monochrome       预处理后强制灰度\n"
    << "  --viewer-ip <ip>         PV31 UDP 目标 IP，默认 192.168.1.50\n"
    << "  --viewer-port <n>        PV31 UDP 目标端口，默认 3335\n"
    << "  --video-serial <path>    图传TX串口路径，默认 /dev/ttyUSB0\n"
    << "  --video-serial-baud <n>  图传TX串口波特率，默认 921600\n"
    << "  --help                   显示帮助\n";
}

uint32_t parse_u32(const std::string & value)
{
  return static_cast<uint32_t>(std::stoul(value, nullptr, 0));
}

Options parse_args(int argc, char ** argv)
{
  Options options;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto require_value = [&](const char * name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(std::string(name) + " 缺少参数");
      }
      return argv[++i];
    };

    if (arg == "--serial") {
      options.serial_port = require_value("--serial");
    } else if (arg == "--baud") {
      throw std::runtime_error("--baud 已移除，当前直接使用 serial 库默认串口配置");
    } else if (arg == "--exposure-ms") {
      options.exposure_ms = std::stod(require_value("--exposure-ms"));
    } else if (arg == "--gain") {
      options.gain = std::stod(require_value("--gain"));
    } else if (arg == "--send-interval-ms") {
      options.send_interval_ms = static_cast<int>(parse_u32(require_value("--send-interval-ms")));
    } else if (arg == "--mode") {
      options.mode = static_cast<uint8_t>(parse_u32(require_value("--mode")));
    } else if (arg == "--sender-id") {
      options.sender_id = static_cast<uint16_t>(parse_u32(require_value("--sender-id")));
    } else if (arg == "--receiver-id") {
      options.receiver_id = static_cast<uint16_t>(parse_u32(require_value("--receiver-id")));
    } else if (arg == "--referee-cmd") {
      options.referee_cmd_id = static_cast<uint16_t>(parse_u32(require_value("--referee-cmd")));
    } else if (arg == "--relay-target") {
      options.referee_cmd_id = map_legacy_relay_target(parse_u32(require_value("--relay-target")));
    } else if (arg == "--payload-text") {
      options.payload_text = require_value("--payload-text");
    } else if (arg == "--0310-telemetry") {
      options.video_0310 = false;
    } else if (arg == "--ffmpeg") {
      options.ffmpeg_path = require_value("--ffmpeg");
    } else if (arg == "--video-size") {
      options.video_size = static_cast<int>(parse_u32(require_value("--video-size")));
    } else if (arg == "--video-fps") {
      options.video_fps = static_cast<int>(parse_u32(require_value("--video-fps")));
    } else if (arg == "--video-bitrate-kbps") {
      options.video_bitrate_kbps = static_cast<int>(parse_u32(require_value("--video-bitrate-kbps")));
    } else if (arg == "--video-gop") {
      options.video_gop = static_cast<int>(parse_u32(require_value("--video-gop")));
    } else if (arg == "--crop-size") {
      options.crop_size = static_cast<int>(parse_u32(require_value("--crop-size")));
    } else if (arg == "--no-static-simplify") {
      options.static_simplify = false;
    } else if (arg == "--motion-threshold") {
      options.motion_threshold = static_cast<int>(parse_u32(require_value("--motion-threshold")));
    } else if (arg == "--motion-erode-px") {
      options.motion_erode_px = static_cast<int>(parse_u32(require_value("--motion-erode-px")));
    } else if (arg == "--motion-dilate-px") {
      options.motion_dilate_px = static_cast<int>(parse_u32(require_value("--motion-dilate-px")));
    } else if (arg == "--motion-trail-frames") {
      options.motion_trail_frames = static_cast<int>(parse_u32(require_value("--motion-trail-frames")));
    } else if (arg == "--trail-disable-motion-ratio") {
      options.trail_disable_motion_ratio = std::stod(require_value("--trail-disable-motion-ratio"));
    } else if (arg == "--bg-update-alpha") {
      options.bg_update_alpha = std::stod(require_value("--bg-update-alpha"));
    } else if (arg == "--bg-blur-sigma") {
      options.bg_blur_sigma = std::stod(require_value("--bg-blur-sigma"));
    } else if (arg == "--center-clear-size") {
      options.center_clear_size = static_cast<int>(parse_u32(require_value("--center-clear-size")));
    } else if (arg == "--force-monochrome") {
      options.force_monochrome = true;
    } else if (arg == "--viewer-ip") {
      options.viewer_ip = require_value("--viewer-ip");
    } else if (arg == "--viewer-port") {
      options.viewer_port = static_cast<int>(parse_u32(require_value("--viewer-port")));
    } else if (arg == "--video-serial") {
      options.video_serial = require_value("--video-serial");
    } else if (arg == "--video-serial-baud") {
      options.video_serial_baud = parse_u32(require_value("--video-serial-baud"));
    } else if (arg == "--help" || arg == "-h") {
      print_help();
      std::exit(0);
    } else {
      throw std::runtime_error("未知参数: " + arg);
    }
  }

  return options;
}

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort() { close(); }

  void open_or_throw(const std::string & port, uint32_t baud_rate = 0)
  {
    close();

    serial_ = std::make_unique<serial::Serial>();
    serial_->setPort(port);

    if (baud_rate > 0) {
      serial_->setBaudrate(baud_rate);
    }

    if (!serial_->isOpen()) {
      serial_->open();
    }
  }

  void close()
  {
    if (serial_ != nullptr) {
      if (serial_->isOpen()) {
        serial_->close();
      }
      serial_.reset();
    }
  }

  void write_all(const uint8_t * data, std::size_t size)
  {
    ensure_open();
    const auto written = serial_->write(data, size);
    if (written != size) {
      if (written == 0) {
        throw std::runtime_error("串口写超时");
      }
      throw std::runtime_error("串口写失败: 仅发送 " + std::to_string(written) + "/" + std::to_string(size) + " 字节");
    }
  }

  bool read_exact(uint8_t * buffer, std::size_t size)
  {
    ensure_open();
    return serial_->read(buffer, size) == size;
  }

private:
  void ensure_open() const
  {
    if (serial_ == nullptr || !serial_->isOpen()) {
      throw std::runtime_error("串口尚未打开");
    }
  }

  std::unique_ptr<serial::Serial> serial_;
};

class UdpSender
{
public:
  UdpSender() = default;
  ~UdpSender() { close(); }

  bool open(const std::string & ip, int port)
  {
    close();
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      std::cerr << "[udp-0310] socket() failed: " << std::strerror(errno) << std::endl;
      return false;
    }
    std::memset(&dest_, 0, sizeof(dest_));
    dest_.sin_family = AF_INET;
    dest_.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, ip.c_str(), &dest_.sin_addr) != 1) {
      std::cerr << "[udp-0310] inet_pton(" << ip << ") failed" << std::endl;
      close();
      return false;
    }
    dest_ip_ = ip;
    dest_port_ = port;
    return true;
  }

  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool send(const uint8_t * data, std::size_t size)
  {
    if (fd_ < 0) { return false; }
    const auto sent = ::sendto(
      fd_, data, size, 0,
      reinterpret_cast<const sockaddr *>(&dest_),
      sizeof(dest_));
    if (sent != static_cast<ssize_t>(size)) {
      std::cerr << "[udp-0310] sendto " << dest_ip_ << ":" << dest_port_
                << " failed, sent=" << sent << " expected=" << size
                << " errno=" << std::strerror(errno) << std::endl;
      return false;
    }
    return true;
  }

private:
  int fd_ = -1;
  sockaddr_in dest_{};
  std::string dest_ip_;
  int dest_port_ = 0;
};

struct FrameInfo
{
  uint16_t width = 0;
  uint16_t height = 0;
  uint32_t bytes = 0;
  uint32_t sequence = 0;
  Clock::time_point timestamp{};
  std::vector<uint8_t> rgb24;
};

struct SharedGimbalState
{
  std::mutex mutex;
  std::optional<bridge::protocol::GimbalToVision> packet;
  Clock::time_point last_rx{};
};

struct GimbalSnapshot
{
  bool online = false;
  bridge::protocol::GimbalToVision packet{};
};

class HikCamera
{
public:
  HikCamera() = default;
  ~HikCamera() { close(); }

  void open_first(double exposure_ms, double gain)
  {
    MV_CC_DEVICE_INFO_LIST device_list;
    std::memset(&device_list, 0, sizeof(device_list));

    const auto ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
    if (ret != MV_OK) {
      throw std::runtime_error("MV_CC_EnumDevices 失败: 0x" + hex_code(ret));
    }

    if (device_list.nDeviceNum == 0) {
      throw std::runtime_error("未找到海康相机");
    }

    check(MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]), "MV_CC_CreateHandle");
    check(MV_CC_OpenDevice(handle_), "MV_CC_OpenDevice");
    check(MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF), "Set ExposureAuto");
    check(MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF), "Set GainAuto");
    check(MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_ms * 1000.0), "Set ExposureTime");
    check(MV_CC_SetFloatValue(handle_, "Gain", gain), "Set Gain");
    check(MV_CC_StartGrabbing(handle_), "MV_CC_StartGrabbing");
    grabbing_ = true;
  }

  bool grab(FrameInfo & frame, unsigned int timeout_ms, bool copy_rgb24)
  {
    MV_FRAME_OUT raw;
    std::memset(&raw, 0, sizeof(raw));

    const auto ret = MV_CC_GetImageBuffer(handle_, &raw, timeout_ms);
    if (ret != MV_OK) {
      return false;
    }

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
      return true;
    }

    check(MV_CC_FreeImageBuffer(handle_, &raw), "MV_CC_FreeImageBuffer");
    return true;
  }

  void close()
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

private:
  static std::string hex_code(unsigned int value)
  {
    std::ostringstream oss;
    oss << std::hex << std::uppercase << value;
    return oss.str();
  }

  static void check(int code, const char * action)
  {
    if (code != MV_OK) {
      throw std::runtime_error(std::string(action) + " 失败: 0x" + hex_code(static_cast<unsigned int>(code)));
    }
  }

  void * handle_ = nullptr;
  bool grabbing_ = false;
};

class H264EncoderProcess
{
public:
  H264EncoderProcess() = default;
  ~H264EncoderProcess() { stop(); }

  bool started() const { return pid_ > 0; }
  uint16_t input_width() const { return input_width_; }
  uint16_t input_height() const { return input_height_; }

  void start(uint16_t width, uint16_t height, const Options & options)
  {
    stop();

    input_width_ = width;
    input_height_ = height;

    int stdin_pipe[2] = {-1, -1};
    int stdout_pipe[2] = {-1, -1};
    int stderr_pipe[2] = {-1, -1};

    if (::pipe(stdin_pipe) != 0 || ::pipe(stdout_pipe) != 0 || ::pipe(stderr_pipe) != 0) {
      close_pair(stdin_pipe);
      close_pair(stdout_pipe);
      close_pair(stderr_pipe);
      throw std::runtime_error("创建 ffmpeg 管道失败: " + std::string(std::strerror(errno)));
    }

    const auto clamped_size = std::clamp(options.video_size, 120, 480);
    const auto clamped_fps = std::clamp(options.video_fps, 10, 60);
    const auto clamped_bitrate = std::clamp(options.video_bitrate_kbps, 40, 110);
    const auto clamped_gop = std::clamp(options.video_gop, clamped_fps, clamped_fps * 12);
    const std::string input_size = std::to_string(width) + "x" + std::to_string(height);
    const std::string video_filter =
      "hqdn3d=4:3:6:4,"
      "eq=contrast=1.12:saturation=0.75:gamma=1.05,"
      "format=yuv420p";

    std::vector<std::string> args = {
      options.ffmpeg_path,
      "-hide_banner",
      "-loglevel", "warning",
      "-f", "rawvideo",
      "-pix_fmt", "bgr24",
      "-s", input_size,
      "-r", std::to_string(clamped_fps),
      "-i", "pipe:0",
      "-an",
      "-vf", video_filter,
      "-c:v", "libx264",
      "-preset", "veryslow",
      "-tune", "zerolatency",
      "-b:v", std::to_string(clamped_bitrate) + "k",
      "-maxrate", std::to_string(std::min(120, clamped_bitrate + 8)) + "k",
      "-bufsize", "24k",
      "-g", std::to_string(clamped_gop),
      "-keyint_min", std::to_string(clamped_gop),
      "-sc_threshold", "0",
      "-bf", "0",
      "-x264-params", "repeat-headers=1:nal-hrd=cbr:force-cfr=1",
      "-pix_fmt", "yuv420p",
      "-f", "h264",
      "pipe:1"};

    const auto child_pid = ::fork();
    if (child_pid < 0) {
      close_pair(stdin_pipe);
      close_pair(stdout_pipe);
      close_pair(stderr_pipe);
      throw std::runtime_error("启动 ffmpeg 失败: " + std::string(std::strerror(errno)));
    }

    if (child_pid == 0) {
      ::dup2(stdin_pipe[0], STDIN_FILENO);
      ::dup2(stdout_pipe[1], STDOUT_FILENO);
      ::dup2(stderr_pipe[1], STDERR_FILENO);
      close_pair(stdin_pipe);
      close_pair(stdout_pipe);
      close_pair(stderr_pipe);

      std::vector<char *> argv;
      argv.reserve(args.size() + 1U);
      for (auto & arg : args) {
        argv.push_back(arg.data());
      }
      argv.push_back(nullptr);
      ::execvp(argv[0], argv.data());
      ::_exit(127);
    }

    pid_ = child_pid;
    close_fd(stdin_pipe[0]);
    close_fd(stdout_pipe[1]);
    close_fd(stderr_pipe[1]);
    stdin_fd_ = stdin_pipe[1];
    stdout_fd_ = stdout_pipe[0];
    stderr_fd_ = stderr_pipe[0];

    stdout_thread_ = std::thread([this] { stdout_loop(); });
    stderr_thread_ = std::thread([this] { stderr_loop(); });

    std::cout << "[video-0310] ffmpeg started input=" << input_size
              << " output=" << clamped_size << "x" << clamped_size
              << " fps=" << clamped_fps
              << " bitrate=" << clamped_bitrate << "kbit/s"
              << " gop=" << clamped_gop << std::endl;
  }

  void stop()
  {
    if (stdin_fd_ >= 0) {
      close_fd(stdin_fd_);
    }

    if (pid_ > 0) {
      ::kill(pid_, SIGTERM);
    }

    if (stdout_thread_.joinable()) {
      stdout_thread_.join();
    }
    if (stderr_thread_.joinable()) {
      stderr_thread_.join();
    }

    if (pid_ > 0) {
      int status = 0;
      ::waitpid(pid_, &status, 0);
      pid_ = -1;
    }

    close_fd(stdout_fd_);
    close_fd(stderr_fd_);

    std::lock_guard<std::mutex> lock(buffer_mutex_);
    encoded_buffer_.clear();
  }

  bool submit_rgb_frame(const std::vector<uint8_t> & frame)
  {
    if (stdin_fd_ < 0 || frame.empty()) {
      return false;
    }

    std::size_t offset = 0;
    while (offset < frame.size()) {
      const auto written = ::write(stdin_fd_, frame.data() + offset, frame.size() - offset);
      if (written > 0) {
        offset += static_cast<std::size_t>(written);
        continue;
      }
      if (written < 0 && errno == EINTR) {
        continue;
      }
      if (written < 0 && errno == EPIPE) {
        return false;
      }
      throw std::runtime_error("写入 ffmpeg stdin 失败: " + std::string(std::strerror(errno)));
    }
    return true;
  }

  bool pop_chunk(
    std::array<uint8_t, bridge::protocol::kCustomClientVideo0310PayloadBytes> & chunk,
    std::size_t & chunk_size)
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (encoded_buffer_.empty()) {
      chunk_size = 0;
      return false;
    }

    chunk_size = std::min(chunk.size(), encoded_buffer_.size());
    std::copy_n(encoded_buffer_.begin(), chunk_size, chunk.begin());
    encoded_buffer_.erase(encoded_buffer_.begin(), encoded_buffer_.begin() + static_cast<long>(chunk_size));
    return true;
  }

  std::size_t queued_bytes() const
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return encoded_buffer_.size();
  }

private:
  static void close_fd(int & fd)
  {
    if (fd >= 0) {
      ::close(fd);
      fd = -1;
    }
  }

  static void close_pair(int (&fds)[2])
  {
    close_fd(fds[0]);
    close_fd(fds[1]);
  }

  void stdout_loop()
  {
    std::array<uint8_t, 4096> buffer{};
    while (true) {
      const auto bytes_read = ::read(stdout_fd_, buffer.data(), buffer.size());
      if (bytes_read > 0) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        encoded_buffer_.insert(encoded_buffer_.end(), buffer.begin(), buffer.begin() + bytes_read);
        continue;
      }
      if (bytes_read < 0 && errno == EINTR) {
        continue;
      }
      break;
    }
  }

  void stderr_loop()
  {
    std::array<char, 512> buffer{};
    std::string line;
    while (true) {
      const auto bytes_read = ::read(stderr_fd_, buffer.data(), buffer.size());
      if (bytes_read > 0) {
        line.append(buffer.data(), buffer.data() + bytes_read);
        std::size_t newline = 0;
        while ((newline = line.find('\n')) != std::string::npos) {
          const auto message = line.substr(0, newline);
          line.erase(0, newline + 1U);
          if (!message.empty()) {
            std::cerr << "[ffmpeg] " << message << std::endl;
          }
        }
        continue;
      }
      if (bytes_read < 0 && errno == EINTR) {
        continue;
      }
      break;
    }
  }

  pid_t pid_ = -1;
  int stdin_fd_ = -1;
  int stdout_fd_ = -1;
  int stderr_fd_ = -1;
  uint16_t input_width_ = 0;
  uint16_t input_height_ = 0;
  std::thread stdout_thread_;
  std::thread stderr_thread_;
  mutable std::mutex buffer_mutex_;
  std::vector<uint8_t> encoded_buffer_;
};

class FramePreprocessor
{
public:
  explicit FramePreprocessor(const Options & options)
  : crop_size_(std::max(0, options.crop_size)),
    output_size_(std::clamp(options.video_size, 120, 480)),
    target_bitrate_kbps_(options.video_bitrate_kbps),
    static_simplify_(options.static_simplify),
    motion_threshold_(std::max(0, options.motion_threshold)),
    motion_erode_px_(std::clamp(options.motion_erode_px, 0, 20)),
    motion_dilate_px_(std::clamp(options.motion_dilate_px, 0, 20)),
    motion_trail_frames_(std::clamp(options.motion_trail_frames, 0, 180)),
    trail_disable_motion_ratio_(std::clamp(options.trail_disable_motion_ratio, 0.0, 1.0)),
    bg_update_alpha_(std::clamp(options.bg_update_alpha, 0.001, 0.2)),
    bg_blur_sigma_(std::max(0.0, options.bg_blur_sigma)),
    center_clear_size_(std::max(0, options.center_clear_size)),
    force_monochrome_(options.force_monochrome)
  {
  }

  int output_size() const { return output_size_; }

  cv::Mat process_rgb24(const FrameInfo & frame)
  {
    if (frame.rgb24.empty() || frame.width == 0 || frame.height == 0) {
      return {};
    }

    cv::Mat input_rgb(
      static_cast<int>(frame.height),
      static_cast<int>(frame.width),
      CV_8UC3,
      const_cast<uint8_t *>(frame.rgb24.data()));
    cv::Mat input_bgr;
    cv::cvtColor(input_rgb, input_bgr, cv::COLOR_RGB2BGR);

    const int crop_edge = crop_size_ > 0
      ? std::min({crop_size_, input_bgr.cols, input_bgr.rows})
      : std::min(input_bgr.cols, input_bgr.rows);
    const int x0 = std::max(0, (input_bgr.cols - crop_edge) / 2);
    const int y0 = std::max(0, (input_bgr.rows - crop_edge) / 2);
    cv::Mat cropped = input_bgr(cv::Rect(x0, y0, crop_edge, crop_edge));

    cv::Mat resized;
    cv::resize(cropped, resized, cv::Size(output_size_, output_size_), 0, 0, cv::INTER_LINEAR);
    cv::Mat working = resized;

    if (force_monochrome_) {
      cv::Mat gray;
      cv::cvtColor(working, gray, cv::COLOR_BGR2GRAY);
      cv::cvtColor(gray, working, cv::COLOR_GRAY2BGR);
    }

    if (!static_simplify_) {
      reset_history();
      return working;
    }

    cv::Mat gray;
    cv::cvtColor(working, gray, cv::COLOR_BGR2GRAY);
    if (background_gray_f32_.empty() || background_gray_f32_.size() != gray.size()) {
      reset_history();
      gray.convertTo(background_gray_f32_, CV_32F);
      return working;
    }

    cv::Mat bg_u8;
    cv::convertScaleAbs(background_gray_f32_, bg_u8);

    cv::Mat diff;
    cv::absdiff(gray, bg_u8, diff);

    cv::Mat motion_mask;
    cv::threshold(diff, motion_mask, motion_threshold_, 255, cv::THRESH_BINARY);
    if (motion_erode_px_ > 0) {
      if (motion_erode_kernel_.empty()) {
        const int kernel_size = 2 * motion_erode_px_ + 1;
        motion_erode_kernel_ = cv::getStructuringElement(
          cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
      }
      cv::erode(motion_mask, motion_mask, motion_erode_kernel_, cv::Point(-1, -1), 1);
    }
    if (motion_dilate_px_ > 0) {
      if (motion_dilate_kernel_.empty()) {
        const int kernel_size = 2 * motion_dilate_px_ + 1;
        motion_dilate_kernel_ = cv::getStructuringElement(
          cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
      }
      cv::dilate(motion_mask, motion_mask, motion_dilate_kernel_, cv::Point(-1, -1), 1);
    }

    const double motion_ratio = motion_mask.total() > 0
      ? static_cast<double>(cv::countNonZero(motion_mask)) / static_cast<double>(motion_mask.total())
      : 0.0;
    const bool suppress_trail = motion_ratio >= trail_disable_motion_ratio_;

    if (center_clear_size_ > 0) {
      const int clear_size = std::min({center_clear_size_, working.cols, working.rows});
      const int clear_x = std::max(0, working.cols / 2 - clear_size / 2);
      const int clear_y = std::max(0, working.rows / 2 - clear_size / 2);
      const int clear_w = std::min(clear_size, working.cols - clear_x);
      const int clear_h = std::min(clear_size, working.rows - clear_y);
      cv::rectangle(motion_mask, cv::Rect(clear_x, clear_y, clear_w, clear_h), cv::Scalar(255), cv::FILLED);
    }

    cv::Mat static_base = working.clone();
    if (!force_monochrome_ && target_bitrate_kbps_ <= 80) {
      cv::Mat gray_static;
      cv::cvtColor(static_base, gray_static, cv::COLOR_BGR2GRAY);
      cv::cvtColor(gray_static, static_base, cv::COLOR_GRAY2BGR);
    }

    cv::Mat blurred_static;
    cv::GaussianBlur(static_base, blurred_static, cv::Size(), bg_blur_sigma_, bg_blur_sigma_);

    cv::Mat focused = blurred_static.clone();
    working.copyTo(focused, motion_mask);

    if (motion_trail_frames_ > 0) {
      motion_mask_history_.push_back(motion_mask.clone());
      trail_frame_history_.push_back(working.clone());
      const auto max_history = static_cast<std::size_t>(motion_trail_frames_ + 1);
      while (motion_mask_history_.size() > max_history) {
        motion_mask_history_.pop_front();
      }
      while (trail_frame_history_.size() > max_history) {
        trail_frame_history_.pop_front();
      }

      const auto history_size = motion_mask_history_.size();
      if (!suppress_trail && history_size > 1 && history_size == trail_frame_history_.size()) {
        cv::Mat trail_mask = motion_mask.clone();
        cv::Mat trail_img = working.clone();
        for (std::size_t i = 0; i + 1 < history_size; ++i) {
          cv::bitwise_or(trail_mask, motion_mask_history_[i], trail_mask);
          cv::max(trail_img, trail_frame_history_[i], trail_img);
        }
        trail_img.copyTo(focused, trail_mask);
      }
    } else {
      motion_mask_history_.clear();
      trail_frame_history_.clear();
    }

    cv::accumulateWeighted(gray, background_gray_f32_, bg_update_alpha_);
    return focused;
  }

private:
  void reset_history()
  {
    motion_mask_history_.clear();
    trail_frame_history_.clear();
    background_gray_f32_.release();
    motion_erode_kernel_.release();
    motion_dilate_kernel_.release();
  }

  int crop_size_ = 0;
  int output_size_ = 300;
  int target_bitrate_kbps_ = 88;
  bool static_simplify_ = true;
  int motion_threshold_ = 14;
  int motion_erode_px_ = 2;
  int motion_dilate_px_ = 6;
  int motion_trail_frames_ = 90;
  double trail_disable_motion_ratio_ = 0.30;
  double bg_update_alpha_ = 0.01;
  double bg_blur_sigma_ = 1.2;
  int center_clear_size_ = 100;
  bool force_monochrome_ = false;
  cv::Mat background_gray_f32_;
  cv::Mat motion_erode_kernel_;
  cv::Mat motion_dilate_kernel_;
  std::deque<cv::Mat> motion_mask_history_;
  std::deque<cv::Mat> trail_frame_history_;
};

std::string format_payload(const Options & options, const FrameInfo & frame, double fps)
{
  std::ostringstream oss;
  oss << options.payload_text << ' '
      << "F=" << frame.sequence << ' '
      << frame.width << 'x' << frame.height << ' '
      << "FPS=" << std::fixed << std::setprecision(1) << fps;
  return oss.str();
}

GimbalSnapshot snapshot_gimbal(SharedGimbalState & state)
{
  std::lock_guard<std::mutex> lock(state.mutex);

  GimbalSnapshot snapshot;
  if (!state.packet.has_value()) {
    return snapshot;
  }

  snapshot.packet = *state.packet;
  snapshot.online = (Clock::now() - state.last_rx) <= kGimbalOnlineTimeout;
  return snapshot;
}

bridge::protocol::VehicleTelemetryV1 build_vehicle_telemetry(
  const Options & options,
  const FrameInfo & frame,
  double fps,
  const GimbalSnapshot & gimbal_snapshot)
{
  bridge::protocol::VehicleTelemetryV1 telemetry{};

  telemetry.unix_ms = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch())
      .count());
  telemetry.frame_seq = frame.sequence;
  telemetry.image_width = frame.width;
  telemetry.image_height = frame.height;
  telemetry.fps_x100 = static_cast<uint16_t>(std::clamp<int>(static_cast<int>(fps * 100.0), 0, 65535));
  telemetry.gain_x100 = static_cast<uint16_t>(
    std::clamp<int>(static_cast<int>(options.gain * 100.0), 0, 65535));
  telemetry.exposure_us = static_cast<uint32_t>(std::max(0.0, options.exposure_ms * 1000.0));

  if (frame.sequence > 0) {
    telemetry.flags |= bridge::protocol::kTelemetryFlagCameraOnline;
  }

  if (gimbal_snapshot.online) {
    telemetry.flags |= bridge::protocol::kTelemetryFlagGimbalOnline;
    telemetry.gimbal_mode = gimbal_snapshot.packet.mode;
    telemetry.bullet_count = gimbal_snapshot.packet.bullet_count;
    telemetry.yaw = gimbal_snapshot.packet.yaw;
    telemetry.yaw_vel = gimbal_snapshot.packet.yaw_vel;
    telemetry.pitch = gimbal_snapshot.packet.pitch;
    telemetry.pitch_vel = gimbal_snapshot.packet.pitch_vel;
    telemetry.bullet_speed = gimbal_snapshot.packet.bullet_speed;
    std::memcpy(telemetry.q, gimbal_snapshot.packet.q, sizeof(telemetry.q));
  }

  bridge::protocol::fill_status_text(
    telemetry.status_text,
    format_payload(options, frame, fps));

  return telemetry;
}

void fill_relay_data(
  bridge::protocol::VisionToGimbalRelay & packet,
  const Options & options,
  const std::string & payload_text,
  const bridge::protocol::VehicleTelemetryV1 & telemetry)
{
  switch (static_cast<bridge::protocol::RelayCommandId>(options.referee_cmd_id)) {
    case bridge::protocol::RelayCommandId::SmallMap0308:
      bridge::protocol::fill_small_map_0308(packet, options.sender_id, options.receiver_id, payload_text);
      return;
    case bridge::protocol::RelayCommandId::CustomController0309:
      bridge::protocol::fill_custom_controller_0309(packet, payload_text);
      return;
    case bridge::protocol::RelayCommandId::CustomClient0310:
      bridge::protocol::fill_custom_client_0310_telemetry(packet, telemetry);
      return;
    default:
      throw std::runtime_error("当前最小工程只支持 0x0308 / 0x0309 / 0x0310");
  }
}

void rx_loop(SerialPort & serial, SharedGimbalState & gimbal_state)
{
  auto last_log = Clock::now();

  while (g_running.load()) {
    try {
      bridge::protocol::GimbalToVision packet{};
      if (!serial.read_exact(packet.head, sizeof(packet.head))) {
        continue;
      }

      if (packet.head[0] != 'S' || packet.head[1] != 'P') {
        continue;
      }

      if (!serial.read_exact(
            reinterpret_cast<uint8_t *>(&packet) + sizeof(packet.head),
            sizeof(packet) - sizeof(packet.head))) {
        continue;
      }

      if (!bridge::protocol::check_crc16(packet)) {
        continue;
      }

      const auto now = Clock::now();
      {
        std::lock_guard<std::mutex> lock(gimbal_state.mutex);
        gimbal_state.packet = packet;
        gimbal_state.last_rx = now;
      }

      if (now - last_log >= std::chrono::seconds(1)) {
        std::cout << "[gimbal-rx] mode=" << static_cast<int>(packet.mode)
                  << " yaw=" << packet.yaw
                  << " pitch=" << packet.pitch
                  << " bullet_speed=" << packet.bullet_speed
                  << " bullet_count=" << packet.bullet_count << std::endl;
        last_log = now;
      }
    } catch (const std::exception & error) {
      std::cerr << "[gimbal-rx] serial error: " << error.what() << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  std::thread receiver;

  try {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGPIPE, SIG_IGN);

    const Options options = parse_args(argc, argv);
    const bool use_0310_video =
      options.video_0310 &&
      options.referee_cmd_id == static_cast<uint16_t>(bridge::protocol::RelayCommandId::CustomClient0310);

    std::cout << "[bridge] opening Hik camera..." << std::endl;
    HikCamera camera;
    camera.open_first(options.exposure_ms, options.gain);
    std::cout << "[bridge] Hik camera ready." << std::endl;

    SerialPort serial;
    SharedGimbalState gimbal_state;
    if (!options.serial_port.empty()) {
      std::cout << "[bridge] opening serial " << options.serial_port << std::endl;
      serial.open_or_throw(options.serial_port);
      std::cout << "[bridge] serial ready." << std::endl;
      receiver = std::thread([&serial, &gimbal_state] { rx_loop(serial, gimbal_state); });
    } else {
      std::cout << "[bridge] no gimbal serial, skipping." << std::endl;
    }

    UdpSender viewer_udp;
    if (use_0310_video && !options.viewer_ip.empty()) {
      if (viewer_udp.open(options.viewer_ip, options.viewer_port)) {
        std::cout << "[bridge] PV31 UDP target: " << options.viewer_ip << ":" << options.viewer_port << std::endl;
      } else {
        std::cerr << "[bridge] WARNING: PV31 UDP not available, video will only go via serial" << std::endl;
      }
    }

    SerialPort video_serial;
    if (use_0310_video && !options.video_serial.empty()) {
      try {
        video_serial.open_or_throw(options.video_serial, options.video_serial_baud);
        std::cout << "[bridge] PV31 video serial TX: " << options.video_serial
                  << " baud=" << options.video_serial_baud << std::endl;
      } catch (const std::exception & error) {
        std::cerr << "[bridge] WARNING: cannot open video serial " << options.video_serial
                  << ": " << error.what() << std::endl;
      }
    }

    FrameInfo latest_frame{};
    FramePreprocessor preprocessor(options);
    H264EncoderProcess video_encoder;
    uint32_t video_sequence = 0;
    bool video_reset = true;
    auto video_stream_start = Clock::now();
    auto last_video_submit = Clock::time_point{};
    uint32_t sent_packets = 0;
    uint32_t frames_in_window = 0;
    double fps = 0.0;
    auto last_send = Clock::now();
    auto last_report = Clock::now();
    auto fps_window = Clock::now();

    uint8_t video_serial_seq = 0;

    std::cout << "[bridge] preprocess crop=" << options.crop_size
          << " output=" << preprocessor.output_size() << 'x' << preprocessor.output_size()
          << " static_simplify=" << (options.static_simplify ? "on" : "off")
          << " trail=" << options.motion_trail_frames
          << " erode=" << options.motion_erode_px
          << " dilate=" << options.motion_dilate_px
          << " mono=" << (options.force_monochrome ? "on" : "off")
          << std::endl;

    while (g_running.load()) {
      const auto now = Clock::now();
      FrameInfo frame{};
      if (camera.grab(frame, 50, use_0310_video)) {
        latest_frame = frame;
        ++frames_in_window;

        if (use_0310_video && !latest_frame.rgb24.empty()) {
          if (!video_encoder.started() ||
              video_encoder.input_width() != static_cast<uint16_t>(preprocessor.output_size()) ||
              video_encoder.input_height() != static_cast<uint16_t>(preprocessor.output_size())) {
            video_encoder.start(
              static_cast<uint16_t>(preprocessor.output_size()),
              static_cast<uint16_t>(preprocessor.output_size()),
              options);
            video_sequence = 0;
            video_reset = true;
            video_stream_start = now;
            last_video_submit = Clock::time_point{};
          }

          const auto frame_interval = std::chrono::milliseconds(
            std::max(1, 1000 / std::clamp(options.video_fps, 1, 120)));
          if (last_video_submit == Clock::time_point{} || now - last_video_submit >= frame_interval) {
            cv::Mat processed = preprocessor.process_rgb24(latest_frame);
            std::vector<uint8_t> processed_bytes;
            if (!processed.empty()) {
              if (!processed.isContinuous()) {
                processed = processed.clone();
              }
              processed_bytes.assign(processed.datastart, processed.dataend);
            }

            if (processed_bytes.empty() || !video_encoder.submit_rgb_frame(processed_bytes)) {
              std::cerr << "[video-0310] ffmpeg stdin closed, restarting encoder" << std::endl;
              video_encoder.start(
                static_cast<uint16_t>(preprocessor.output_size()),
                static_cast<uint16_t>(preprocessor.output_size()),
                options);
              video_sequence = 0;
              video_reset = true;
              video_stream_start = now;
            }
            last_video_submit = now;
          }
        }
      }

      const auto fps_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_window).count();
      if (fps_elapsed >= 1000) {
        fps = frames_in_window * 1000.0 / static_cast<double>(fps_elapsed);
        frames_in_window = 0;
        fps_window = now;
      }

      if (now - last_send >= std::chrono::milliseconds(options.send_interval_ms)) {
        bridge::protocol::VisionToGimbalRelay packet{};
        packet.mode = options.mode;
        const auto gimbal_snapshot = snapshot_gimbal(gimbal_state);
        bool packet_ready = true;

        if (use_0310_video) {
          std::array<uint8_t, bridge::protocol::kCustomClientVideo0310PayloadBytes> video_chunk{};
          std::size_t video_chunk_size = 0;
          packet_ready = video_encoder.pop_chunk(video_chunk, video_chunk_size);
          if (packet_ready) {
            const auto stream_ms = static_cast<uint32_t>(
              std::chrono::duration_cast<std::chrono::milliseconds>(now - video_stream_start).count());
            const auto flags = video_reset ? 1U : 0U;
            video_reset = false;
            bridge::protocol::fill_custom_client_0310_video_chunk(
              packet,
              bridge::protocol::kCustomClientVideo0310CodecH264,
              static_cast<uint8_t>(flags),
              video_sequence++,
              stream_ms,
              video_chunk.data(),
              video_chunk_size);
          }
        } else {
          const auto payload_text = format_payload(options, latest_frame, fps);
          const auto telemetry = build_vehicle_telemetry(options, latest_frame, fps, gimbal_snapshot);
          fill_relay_data(packet, options, payload_text, telemetry);
        }

        if (packet_ready) {
          bridge::protocol::finalize_crc16(packet);
          if (!options.serial_port.empty()) {
            serial.write_all(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
          }
          if (use_0310_video && packet.relay_data_length > 0) {
            try {
              uint8_t ref_frame[320];
              std::size_t ref_len = 0;
              bridge::protocol::build_referee_frame(
                ref_frame, ref_len,
                packet.referee_cmd_id,
                video_serial_seq++,
                packet.relay_data,
                packet.relay_data_length);
              video_serial.write_all(ref_frame, ref_len);
            } catch (const std::exception & error) {
              static auto last_serial_err = Clock::now();
              const auto now_err = Clock::now();
              if (now_err - last_serial_err >= std::chrono::seconds(5)) {
                std::cerr << "[video-serial] write error: " << error.what() << std::endl;
                last_serial_err = now_err;
              }
            }
            viewer_udp.send(packet.relay_data, packet.relay_data_length);
          }
          ++sent_packets;
          last_send = now;
        }
      }

      if (now - last_report >= std::chrono::seconds(1)) {
        const auto gimbal_snapshot = snapshot_gimbal(gimbal_state);
        std::cout << "[bridge] frame_seq=" << latest_frame.sequence
                  << " resolution=" << latest_frame.width << 'x' << latest_frame.height
                  << " fps=" << std::fixed << std::setprecision(1) << fps
                  << " gimbal=" << (gimbal_snapshot.online ? "online" : "waiting")
                  << " sent=" << sent_packets;
        if (use_0310_video) {
          std::cout << " video_backlog=" << video_encoder.queued_bytes()
                    << " video_seq=" << video_sequence;
        }
        std::cout << std::endl;
        last_report = now;
      }
    }

    g_running = false;
    if (receiver.joinable()) {
      receiver.join();
    }
    return 0;
  } catch (const std::exception & error) {
    g_running = false;
    if (receiver.joinable()) {
      receiver.join();
    }
    std::cerr << "[bridge] fatal: " << error.what() << std::endl;
    return 1;
  }
}