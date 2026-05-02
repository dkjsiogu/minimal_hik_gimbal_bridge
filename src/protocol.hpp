#pragma once

#include <algorithm>
#include <array>
#include <codecvt>
#include <cstdint>
#include <cstring>
#include <locale>
#include <string>
#include <string_view>

namespace bridge::protocol
{
enum class RelayCommandId : uint16_t
{
  SmallMap0308 = 0x0308,
  CustomController0309 = 0x0309,
  CustomClient0310 = 0x0310,
};

inline constexpr uint32_t kTelemetryFlagCameraOnline = 1U << 0U;
inline constexpr uint32_t kTelemetryFlagGimbalOnline = 1U << 1U;
inline constexpr uint8_t kCustomClientVideo0310Version = 1U;
inline constexpr uint8_t kCustomClientVideo0310CodecH264 = 1U;
inline constexpr uint8_t kCustomClientVideo0310CodecHevc = 2U;
inline constexpr std::size_t kCustomClientVideo0310HeaderBytes = 24U;
inline constexpr std::size_t kCustomClientVideo0310PayloadBytes = 276U;

struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode = 0;
  float q[4] = {1.0F, 0.0F, 0.0F, 0.0F};
  float yaw = 0.0F;
  float yaw_vel = 0.0F;
  float pitch = 0.0F;
  float pitch_vel = 0.0F;
  float bullet_speed = 0.0F;
  uint16_t bullet_count = 0;
  uint16_t crc16 = 0;
};

struct __attribute__((packed)) RefereeSmallMap0308
{
  uint16_t sender_id = 0;
  uint16_t receiver_id = 0;
  uint8_t user_data[30] = {0};
};

struct __attribute__((packed)) RefereeCustomController0309
{
  uint8_t data[30] = {0};
};

struct __attribute__((packed)) RefereeCustomClient0310
{
  uint8_t data[300] = {0};
};

struct __attribute__((packed)) CustomClientVideo0310Chunk
{
  uint8_t magic[4] = {'P', 'V', '3', '1'};
  uint8_t version = kCustomClientVideo0310Version;
  uint8_t header_bytes = kCustomClientVideo0310HeaderBytes;
  uint8_t codec = kCustomClientVideo0310CodecH264;
  uint8_t flags = 0;
  uint32_t sequence = 0;
  uint32_t stream_ms = 0;
  uint16_t payload_bytes = 0;
  uint16_t payload_checksum = 0;
  uint32_t reserved0 = 0;
  uint8_t payload[kCustomClientVideo0310PayloadBytes] = {0};
};

struct __attribute__((packed)) VehicleTelemetryV1
{
  uint8_t magic[4] = {'P', 'D', 'L', '1'};
  uint16_t version = 1;
  uint16_t struct_bytes = sizeof(VehicleTelemetryV1);
  uint32_t flags = 0;

  uint64_t unix_ms = 0;
  uint32_t frame_seq = 0;
  uint16_t image_width = 0;
  uint16_t image_height = 0;
  uint16_t fps_x100 = 0;
  uint16_t gain_x100 = 0;
  uint32_t exposure_us = 0;

  uint8_t gimbal_mode = 0;
  uint8_t reserved0 = 0;
  uint16_t bullet_count = 0;

  float yaw = 0.0F;
  float yaw_vel = 0.0F;
  float pitch = 0.0F;
  float pitch_vel = 0.0F;
  float bullet_speed = 0.0F;
  float q[4] = {1.0F, 0.0F, 0.0F, 0.0F};

  uint8_t status_text[64] = {0};
};

inline constexpr std::size_t kRelayDataMaxBytes = sizeof(RefereeCustomClient0310);

struct __attribute__((packed)) VisionToGimbalRelay
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode = 0;
  float yaw = 0.0F;
  float yaw_vel = 0.0F;
  float yaw_acc = 0.0F;
  float pitch = 0.0F;
  float pitch_vel = 0.0F;
  float pitch_acc = 0.0F;

  uint16_t referee_cmd_id = static_cast<uint16_t>(RelayCommandId::CustomClient0310);
  uint16_t relay_data_length = 0;
  uint8_t relay_data[kRelayDataMaxBytes] = {0};
  uint16_t crc16 = 0;
};

static_assert(sizeof(GimbalToVision) == 43, "Unexpected GimbalToVision size");
static_assert(sizeof(RefereeSmallMap0308) == 34, "Unexpected RefereeSmallMap0308 size");
static_assert(sizeof(RefereeCustomController0309) == 30, "Unexpected RefereeCustomController0309 size");
static_assert(sizeof(RefereeCustomClient0310) == 300, "Unexpected RefereeCustomClient0310 size");
static_assert(sizeof(CustomClientVideo0310Chunk) == sizeof(RefereeCustomClient0310), "Unexpected 0310 video chunk size");
static_assert(sizeof(VehicleTelemetryV1) <= sizeof(RefereeCustomClient0310), "VehicleTelemetryV1 too large");
static_assert(sizeof(VisionToGimbalRelay) == 333, "Unexpected VisionToGimbalRelay size");

inline constexpr uint16_t kCrc16Init = 0xFFFF;
inline constexpr std::array<uint16_t, 256> kCrc16Table = {
  0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF, 0x8C48, 0x9DC1, 0xAF5A,
  0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7, 0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C,
  0x75B7, 0x643E, 0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876, 0x2102,
  0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD, 0xAD4A, 0xBCC3, 0x8E58, 0x9FD1,
  0xEB6E, 0xFAE7, 0xC87C, 0xD9F5, 0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5,
  0x453C, 0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974, 0x4204, 0x538D,
  0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB, 0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868,
  0x99E1, 0xAB7A, 0xBAF3, 0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
  0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72, 0x6306, 0x728F, 0x4014,
  0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9, 0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3,
  0x8A78, 0x9BF1, 0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738, 0xFFCF,
  0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70, 0x8408, 0x9581, 0xA71A, 0xB693,
  0xC22C, 0xD3A5, 0xE13E, 0xF0B7, 0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76,
  0x7CFF, 0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036, 0x18C1, 0x0948,
  0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E, 0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E,
  0xF2A7, 0xC03C, 0xD1B5, 0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
  0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134, 0x39C3, 0x284A, 0x1AD1,
  0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C, 0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1,
  0xA33A, 0xB2B3, 0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB, 0xD68D,
  0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232, 0x5AC5, 0x4B4C, 0x79D7, 0x685E,
  0x1CE1, 0x0D68, 0x3FF3, 0x2E7A, 0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238,
  0x93B1, 0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9, 0xF78F, 0xE606,
  0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330, 0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3,
  0x2C6A, 0x1EF1, 0x0F78};

inline uint16_t crc16(const uint8_t * data, uint32_t len)
{
  uint16_t crc = kCrc16Init;
  while (len-- != 0U) {
    const auto byte = *data++;
    const auto index = static_cast<uint8_t>((crc ^ byte) & 0x00FFU);
    crc = static_cast<uint16_t>((crc >> 8U) ^ kCrc16Table[index]);
  }
  return crc;
}

inline uint16_t checksum16(const uint8_t * data, std::size_t len)
{
  uint16_t sum = 0;
  for (std::size_t i = 0; i < len; ++i) {
    sum = static_cast<uint16_t>(sum + data[i]);
  }
  return sum;
}

template <typename Packet>
inline void finalize_crc16(Packet & packet)
{
  packet.crc16 = crc16(reinterpret_cast<const uint8_t *>(&packet), sizeof(Packet) - sizeof(packet.crc16));
}

template <typename Packet>
inline bool check_crc16(const Packet & packet)
{
  return crc16(reinterpret_cast<const uint8_t *>(&packet), sizeof(Packet) - sizeof(packet.crc16)) == packet.crc16;
}

inline uint16_t clamp_relay_size(std::size_t size)
{
  return static_cast<uint16_t>(std::min(size, kRelayDataMaxBytes));
}

inline void set_relay_payload(
  VisionToGimbalRelay & packet, RelayCommandId cmd_id, const uint8_t * data, std::size_t size)
{
  packet.referee_cmd_id = static_cast<uint16_t>(cmd_id);
  packet.relay_data_length = clamp_relay_size(size);
  std::memset(packet.relay_data, 0, sizeof(packet.relay_data));
  std::memcpy(packet.relay_data, data, packet.relay_data_length);
}

inline void fill_small_map_utf16le(uint8_t (&user_data)[30], std::string_view text)
{
  std::memset(user_data, 0, sizeof(user_data));

  try {
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> converter;
    const auto utf16 = converter.from_bytes(text.data(), text.data() + text.size());
    const auto max_chars = sizeof(user_data) / 2;
    const auto count = std::min<std::size_t>(utf16.size(), max_chars);
    for (std::size_t i = 0; i < count; ++i) {
      const auto code = static_cast<uint16_t>(utf16[i]);
      user_data[i * 2] = static_cast<uint8_t>(code & 0x00FFU);
      user_data[i * 2 + 1] = static_cast<uint8_t>((code >> 8U) & 0x00FFU);
    }
  } catch (...) {
    const auto copy_size = std::min(text.size(), sizeof(user_data));
    std::memcpy(user_data, text.data(), copy_size);
  }
}

inline void fill_small_map_0308(
  VisionToGimbalRelay & packet, uint16_t sender_id, uint16_t receiver_id, std::string_view text)
{
  RefereeSmallMap0308 payload{};
  payload.sender_id = sender_id;
  payload.receiver_id = receiver_id;
  fill_small_map_utf16le(payload.user_data, text);
  set_relay_payload(
    packet,
    RelayCommandId::SmallMap0308,
    reinterpret_cast<const uint8_t *>(&payload),
    sizeof(payload));
}

inline void fill_custom_controller_0309(VisionToGimbalRelay & packet, std::string_view text)
{
  RefereeCustomController0309 payload{};
  const auto copy_size = std::min(text.size(), sizeof(payload.data));
  std::memcpy(payload.data, text.data(), copy_size);
  set_relay_payload(
    packet,
    RelayCommandId::CustomController0309,
    reinterpret_cast<const uint8_t *>(&payload),
    sizeof(payload));
}

inline void fill_custom_client_0310(VisionToGimbalRelay & packet, std::string_view text)
{
  RefereeCustomClient0310 payload{};
  const auto copy_size = std::min(text.size(), sizeof(payload.data));
  std::memcpy(payload.data, text.data(), copy_size);
  set_relay_payload(
    packet,
    RelayCommandId::CustomClient0310,
    reinterpret_cast<const uint8_t *>(&payload),
    sizeof(payload));
}

inline void fill_status_text(uint8_t (&buffer)[64], std::string_view text)
{
  std::memset(buffer, 0, sizeof(buffer));
  const auto copy_size = std::min(text.size(), sizeof(buffer) - 1U);
  std::memcpy(buffer, text.data(), copy_size);
}

inline void fill_custom_client_0310_telemetry(
  VisionToGimbalRelay & packet, const VehicleTelemetryV1 & telemetry)
{
  RefereeCustomClient0310 payload{};
  std::memcpy(payload.data, &telemetry, sizeof(telemetry));
  set_relay_payload(
    packet,
    RelayCommandId::CustomClient0310,
    reinterpret_cast<const uint8_t *>(&payload),
    sizeof(payload));
}

inline void fill_custom_client_0310_video_chunk(
  VisionToGimbalRelay & packet,
  uint8_t codec,
  uint8_t flags,
  uint32_t sequence,
  uint32_t stream_ms,
  const uint8_t * data,
  std::size_t size)
{
  CustomClientVideo0310Chunk payload{};
  payload.codec = codec;
  payload.flags = flags;
  payload.sequence = sequence;
  payload.stream_ms = stream_ms;
  payload.payload_bytes = static_cast<uint16_t>(std::min(size, sizeof(payload.payload)));
  if (payload.payload_bytes > 0U) {
    std::memcpy(payload.payload, data, payload.payload_bytes);
  }
  payload.payload_checksum = checksum16(payload.payload, payload.payload_bytes);

  set_relay_payload(
    packet,
    RelayCommandId::CustomClient0310,
    reinterpret_cast<const uint8_t *>(&payload),
    sizeof(payload));
}
}  // namespace bridge::protocol