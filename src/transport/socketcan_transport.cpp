#include <array>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include "socketcan_transport.hpp"

using kot_motor::transport::SocketCanTransport;

SocketCanTransport::SocketCanTransport(const std::string& canName) 
  : sock()
  , ifname(canName)
{}

SocketCanTransport::~SocketCanTransport() {
  if (sock.has_value()) {
    this->close();
  }
}

SocketCanTransport::Status SocketCanTransport::open() {
  // creates an endpoint for communication
  // returns a socket descriptor
  int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (s == -1) {
    return Status::FAIL;
  }
  
  // control device-specific operations
  struct ifreq ifr;
  std::strcpy(ifr.ifr_name, this->ifname.data());
  if (ioctl(s, SIOCGIFINDEX, &ifr) == -1) {
    ::close(s);
    return Status::FAIL;
  }
  
  // associates a socket with an address 
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(s, reinterpret_cast<struct sockaddr*>(&addr), 
    sizeof(addr)) == -1) 
  {
    ::close(s);
    return Status::FAIL;
  }

  sock = s;

  return Status::SUCCESS_INIT;
}

SocketCanTransport::Status SocketCanTransport::close() {
  ::close(int(sock.value()));
  sock.reset();
  return Status::SUCCESS;
}

SocketCanTransport::Status SocketCanTransport::write(const CanFrame & canFrame) {
  if (!sock.has_value()) {
    return Status::FAIL;
  }

  struct can_frame frame;
  frame.can_id = canFrame.canId;
  frame.len = canFrame.size;

  std::memcpy(frame.data, canFrame.data, canFrame.size);

  int nbytes = ::write(sock.value(), &frame, sizeof(struct can_frame));
  if (nbytes == -1) {
    return Status::FAIL;
  } else {
    return Status::SUCCESS;
  }
}

std::optional<SocketCanTransport::CanFrame> SocketCanTransport::read() {
  // TODO
  if (sock.has_value()) {
    return {};
  }
  return {};
}

const std::string& SocketCanTransport::canInterfaceName() const {
  return ifname;
}




