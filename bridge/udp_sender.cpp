#include "bridge/udp_sender.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <unistd.h>

namespace bridge
{

UdpSender::UdpSender() = default;

UdpSender::~UdpSender()
{
  close();
}

bool UdpSender::open(const std::string & ip, int port)
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

void UdpSender::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool UdpSender::send(const uint8_t * data, std::size_t size)
{
  if (fd_ < 0) {
    return false;
  }
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

}  // namespace bridge
