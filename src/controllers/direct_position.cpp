#include "direct_position.hpp"

using kot_motor::controller::DirectPositionController;
using namespace kot_motor::dimensions;

DirectPositionController::DirectPositionController(
  Motor & motor, const UserLimits & userLimits
) noexcept
  : BasicController(motor), userLimits(userLimits)
{ }

DirectPositionController::Status DirectPositionController::reset() noexcept
{
  motor.position(0);
  motor.stiffness(0);
  motor.damper(0);

  return Status::SUCCESS;
}

/*********************** Control parameters setters ************************/

DirectPositionController::Status DirectPositionController::position(Radian pos) noexcept
{
  if (motor.state() == Motor::MotorState::MOTOR_MODE_NOT_ACTIVE) {
    return Status::MOTOR_NOT_SWITCHED_ON;
  } else if (pos == motor.position()) {
    return Status::ALREADY_DONE;
  }

  auto && limits =
    userLimits.position.value_or(motor.motorInfo().motorHwLimits.position);

  pos = dimensions::limitUnitBy(pos, limits.min, limits.max);

  motor.position(pos);
  BasicTransport::Status canStatus = motor.sendToMotor();

  switch (canStatus) {
    case BasicTransport::Status::SUCCESS:
      return Status::SUCCESS;
    default:
      return Status::FAIL;
  }
}

DirectPositionController::Status
  DirectPositionController::stiffeness(RotationalStiffness stiff) noexcept
{
  if (motor.state() == Motor::MotorState::MOTOR_MODE_NOT_ACTIVE) {
    return Status::MOTOR_NOT_SWITCHED_ON;
  } else if (stiff == motor.stiffness()) {
    return Status::ALREADY_DONE;
  }

  auto && limits =
    userLimits.stiffeness.value_or(motor.motorInfo().motorHwLimits.stiffness);

  stiff = dimensions::limitUnitBy(stiff, limits.min, limits.max);

  motor.stiffness(stiff);
  BasicTransport::Status canStatus = motor.sendToMotor();

  switch (canStatus) {
    case BasicTransport::Status::SUCCESS:
      return Status::SUCCESS;
    default:
      return Status::FAIL;
  }
}

DirectPositionController::Status
  DirectPositionController::damper(RotationalDamping damp) noexcept
{
  if (motor.state() == Motor::MotorState::MOTOR_MODE_NOT_ACTIVE) {
    return Status::MOTOR_NOT_SWITCHED_ON;
  } else if (damp == motor.damper()) {
    return Status::ALREADY_DONE;
  }

  auto && limits =
    userLimits.damper.value_or(motor.motorInfo().motorHwLimits.damper);

  damp = dimensions::limitUnitBy(damp, limits.min, limits.max);

  motor.damper(damp);
  BasicTransport::Status canStatus = motor.sendToMotor();

  switch (canStatus) {
    case BasicTransport::Status::SUCCESS:
      return Status::SUCCESS;
    default:
      return Status::FAIL;
  }
}

/*********************** Control parameters getters ************************/

Radian DirectPositionController::position() const noexcept
{
  return motor.position();
}

RotationalStiffness DirectPositionController::stiffeness() const noexcept
{
  return motor.stiffness();
}

RotationalDamping DirectPositionController::damper() const noexcept
{
  return motor.damper();
}




