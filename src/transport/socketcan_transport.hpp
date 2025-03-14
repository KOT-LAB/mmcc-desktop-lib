#ifndef SOCKETCAN_TRANSPORT_HPP
#define SOCKETCAN_TRANSPORT_HPP

#include <string>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <optional>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "basic_transport.hpp"

namespace kot_motor::transport {

class SocketCanTransport : public BasicTransport {
public:
  SocketCanTransport(const std::string& canName);
  ~SocketCanTransport();

  Status open();
  Status close();
  Status write(const CanFrame & canFrame) override;
  std::optional<CanFrame> read() override;
  const std::string& canInterfaceName() const;

private:
  std::optional<int> sock; // file descriptor
  const std::string ifname;
};

} // namespace kot_motor::transport

#endif // SOCKETCAN_TRANSPORT_HPP




