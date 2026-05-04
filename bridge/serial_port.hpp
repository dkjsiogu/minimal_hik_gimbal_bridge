#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace serial
{
class Serial;
}

namespace bridge
{

class SerialPort
{
public:
  SerialPort();
  ~SerialPort();

  void open_or_throw(const std::string & port, uint32_t baud_rate = 0, uint32_t timeout_ms = 20);
  bool is_open() const;
  void close();
  void write_all(const uint8_t * data, std::size_t size);
  bool read_exact(uint8_t * buffer, std::size_t size);

private:
  void ensure_open_locked() const;
  void close_locked();

  mutable std::mutex mutex_;
  std::unique_ptr<serial::Serial> serial_;
};

}  // namespace bridge
