#include "bridge/common.hpp"

namespace bridge
{

bool should_log_at_interval(
  Clock::time_point now,
  Clock::time_point & last_log,
  Clock::duration interval)
{
  if (last_log == Clock::time_point{} || now - last_log >= interval) {
    last_log = now;
    return true;
  }
  return false;
}

}  // namespace bridge
