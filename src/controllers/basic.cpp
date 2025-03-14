#include "basic.hpp"
#include <algorithm>

using kot_motor::controller::BasicController;
using kot_motor::motor::Motor;

BasicController::BasicController(Motor & motor) noexcept
  : motor(motor)
{ }

BasicController::~BasicController() = default;

const Motor & BasicController::getMotor() const noexcept
{
  return motor;
}

Motor::MotorState BasicController::getMotorState() const noexcept
{
  return motor.state();
}

BasicController::Status BasicController::switchOnSetZero() noexcept
{
  auto check = [](BasicTransport::Status status) {
    return status == BasicTransport::Status::SUCCESS;
  };

  std::array<BasicTransport::Status, 3> statuses;
  statuses[0] = motor.enterMotorMode();
  statuses[1] = motor.setProgramZero();
  statuses[2] = motor.resetParameters();

  if (std::all_of(statuses.begin(), statuses.end(), check)) {
    return Status::SUCCESS;
  } else {
    return Status::FAIL;
  }
}

BasicController::Status BasicController::switchOReturnToPrevZero() noexcept
{
  BasicTransport::Status status = motor.enterMotorMode();

  if (status == BasicTransport::Status::SUCCESS) {
    return Status::SUCCESS;
  } else {
    return Status::FAIL;
  }
}

BasicController::Status BasicController::switchOn(SwitchOnMode mode) noexcept
{
  if (motor.state() == Motor::MotorState::MOTOR_MODE_ACTIVE) {
    return Status::ALREADY_DONE;
  }

  switch (mode) {
    case SwitchOnMode::SET_NEW_ZERO:
      return switchOnSetZero();
    case SwitchOnMode::RETURN_TO_FREV_ZERO:
      return switchOReturnToPrevZero();
    default:
      return Status::FAIL;
  }
}

BasicController::Status BasicController::switchOff() noexcept
{
  if (motor.state() == Motor::MotorState::MOTOR_MODE_NOT_ACTIVE) {
    return Status::ALREADY_DONE;
  }

  BasicTransport::Status canStatus = motor.exitMotorMode();

  switch (canStatus) {
    case BasicTransport::Status::SUCCESS:
      return Status::SUCCESS;
    default:
      return Status::FAIL;
  }
}

BasicController::Status BasicController::toggleState() noexcept
{
  if (motor.state() == Motor::MotorState::MOTOR_MODE_ACTIVE) {
    return switchOff();
  } else {
    return switchOn();
  }
}




