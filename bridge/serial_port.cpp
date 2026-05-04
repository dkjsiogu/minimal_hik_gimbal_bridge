#include "bridge/serial_port.hpp"

#include "serial/serial.h"

#include <stdexcept>

namespace bridge
{

SerialPort::SerialPort() = default;

SerialPort::~SerialPort()
{
  close();
}

void SerialPort::open_or_throw(const std::string & port, uint32_t baud_rate, uint32_t timeout_ms)
{
  std::lock_guard<std::mutex> lock(mutex_);
  close_locked();

  serial_ = std::make_unique<serial::Serial>();
  serial_->setPort(port);

  if (baud_rate > 0) {
    serial_->setBaudrate(baud_rate);
  }

  auto timeout = serial::Timeout::simpleTimeout(timeout_ms);
  serial_->setTimeout(timeout);

  if (!serial_->isOpen()) {
    serial_->open();
  }
}

bool SerialPort::is_open() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return serial_ != nullptr && serial_->isOpen();
}

void SerialPort::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  close_locked();
}

void SerialPort::write_all(const uint8_t * data, std::size_t size)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ensure_open_locked();
  try {
    const auto written = serial_->write(data, size);
    if (written != size) {
      if (written == 0) {
        throw std::runtime_error("串口写超时");
      }
      throw std::runtime_error("串口写失败: 仅发送 " + std::to_string(written) + "/" + std::to_string(size) + " 字节");
    }
  } catch (...) {
    close_locked();
    throw;
  }
}

bool SerialPort::read_exact(uint8_t * buffer, std::size_t size)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ensure_open_locked();
  try {
    return serial_->read(buffer, size) == size;
  } catch (...) {
    close_locked();
    throw;
  }
}

void SerialPort::ensure_open_locked() const
{
  if (serial_ == nullptr || !serial_->isOpen()) {
    throw std::runtime_error("串口尚未打开");
  }
}

void SerialPort::close_locked()
{
  if (serial_ != nullptr) {
    try {
      if (serial_->isOpen()) {
        serial_->close();
      }
    } catch (...) {
    }
    serial_.reset();
  }
}

}  // namespace bridge
