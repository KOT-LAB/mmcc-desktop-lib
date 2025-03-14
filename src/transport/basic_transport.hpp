#ifndef BASIC_TRANSPORT_HPP
#define BASIC_TRANSPORT_HPP

#include <stdint.h>
#include <optional>

namespace kot_motor::transport {

class BasicTransport {
public:
  enum class Status : uint8_t {
    SUCCESS,
    FAIL,
    INIT_FAIL,
    SUCCESS_INIT
  };

  struct CanFrame {
    uint8_t canId;
    uint8_t masterCanId;
    uint8_t data[8]; // TODO default size == 8 ?
    uint8_t size;

    CanFrame();

    CanFrame(
      uint8_t canId,
      uint8_t masterCanId,
      const uint8_t * data,
      uint8_t size = 8
    );
  };

public:
  virtual ~BasicTransport();
  virtual Status write(const CanFrame & canFrame) = 0;
  virtual std::optional<CanFrame> read() = 0;
};

} // namespace kot_motor::transport

#endif // BASIC_TRANSPORT_HPP
