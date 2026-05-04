#pragma once

#include <chrono>

namespace bridge
{

using Clock = std::chrono::steady_clock;

inline constexpr auto kGimbalOnlineTimeout = std::chrono::milliseconds(1000);
inline constexpr auto kDeviceReconnectInterval = std::chrono::milliseconds(200);
inline constexpr auto kReconnectLogInterval = std::chrono::seconds(2);
inline constexpr const char * kDefaultConfigPath = "config/bridge.yaml";

bool should_log_at_interval(
  Clock::time_point now,
  Clock::time_point & last_log,
  Clock::duration interval);

}  // namespace bridge
