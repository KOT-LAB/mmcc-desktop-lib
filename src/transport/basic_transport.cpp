#include <cstring>
#include "basic_transport.hpp"

using kot_motor::transport::BasicTransport;

BasicTransport::CanFrame::CanFrame() = default;

BasicTransport::~BasicTransport() = default;

BasicTransport::CanFrame::CanFrame(
  uint8_t canId, uint8_t masterCanId, const uint8_t * data, uint8_t size
)
  : canId(canId), masterCanId(masterCanId), data(), size(size)
{
  std::memcpy(this->data, data, size);
}




