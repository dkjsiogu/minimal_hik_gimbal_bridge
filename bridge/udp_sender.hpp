#pragma once

#include <cstddef>
#include <cstdint>
#include <netinet/in.h>
#include <string>

namespace bridge
{

class UdpSender
{
public:
  UdpSender();
  ~UdpSender();

  bool open(const std::string & ip, int port);
  void close();
  bool send(const uint8_t * data, std::size_t size);

private:
  int fd_ = -1;
  sockaddr_in dest_{};
  std::string dest_ip_;
  int dest_port_ = 0;
};

}  // namespace bridge
