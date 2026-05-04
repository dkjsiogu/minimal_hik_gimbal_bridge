#pragma once

#include "bridge/common.hpp"

#include <cstdint>
#include <vector>

namespace bridge
{

struct FrameInfo
{
  uint16_t width = 0;
  uint16_t height = 0;
  uint32_t bytes = 0;
  uint32_t sequence = 0;
  Clock::time_point timestamp{};
  std::vector<uint8_t> rgb24;
};

FrameInfo make_test_pattern_frame(uint32_t sequence, Clock::time_point timestamp);

}  // namespace bridge
