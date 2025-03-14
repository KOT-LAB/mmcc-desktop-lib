#ifndef BASIC_CONTROLLER_HPP
#define BASIC_CONTROLLER_HPP


#include "motor/motor.hpp"
#include "dimensions/dimensions.hpp"

namespace kot_motor::controller {

using motor::Motor;
using transport::BasicTransport;

// TODO copy/move assignments operators?
class BasicController {
public:
  enum class Status : uint8_t {
    SUCCESS,
    FAIL,
    ALREADY_DONE,
    MOTOR_NOT_SWITCHED_ON
  };

  enum class SwitchOnMode {
    SET_NEW_ZERO,
    RETURN_TO_FREV_ZERO
  };

public:
  BasicController(Motor & motor) noexcept;
  BasicController(const BasicController &) = default;
  BasicController(BasicController &&) = default;
  BasicController & operator=(const BasicController &) = delete;
  BasicController & operator=(BasicController &&) = delete;
  virtual ~BasicController();

  virtual const Motor & getMotor() const noexcept;
  virtual Motor::MotorState getMotorState() const noexcept;

  virtual Status switchOn(SwitchOnMode mode = SwitchOnMode::SET_NEW_ZERO) noexcept;
  virtual Status switchOff() noexcept;
  virtual Status toggleState() noexcept;

  virtual Status reset() noexcept = 0;

protected:
  virtual Status switchOnSetZero() noexcept;
  virtual Status switchOReturnToPrevZero() noexcept;

protected:
  Motor & motor;
};

} // namespace kot_motor::controller

#endif // BASIC_CONTROLLER_HPP




