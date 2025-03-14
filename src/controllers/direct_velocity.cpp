#include "direct_velocity.hpp"

using kot_motor::controller::DirectVelocityController;
using namespace kot_motor::dimensions;

DirectVelocityController::DirectVelocityController(
  Motor & motor, const UserLimits & userLimits
) noexcept
  : BasicController(motor), userLimits(userLimits)
{ }

DirectVelocityController::Status DirectVelocityController::reset() noexcept
{
  motor.velocity(0);
  motor.damper(0);

  return Status::SUCCESS;
}

/*********************** Control parameters setters ************************/

DirectVelocityController::Status
  DirectVelocityController::velocity(AngularVelocity vel) noexcept
{
  if (motor.state() == Motor::MotorState::MOTOR_MODE_NOT_ACTIVE) {
    return Status::MOTOR_NOT_SWITCHED_ON;
  } else if (vel == motor.velocity()) {
    return Status::ALREADY_DONE;
  }

  auto && limits =
    userLimits.velocity.value_or(motor.motorInfo().motorHwLimits.velocity);

  vel = dimensions::limitUnitBy(vel, limits.min, limits.max);

  motor.velocity(vel);
  BasicTransport::Status canStatus = motor.sendToMotor();

  switch (canStatus) {
    case BasicTransport::Status::SUCCESS:
      return Status::SUCCESS;
    default:
      return Status::FAIL;
  }
}

DirectVelocityController::Status
  DirectVelocityController::damper(RotationalDamping damp) noexcept
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

DirectVelocityController::Status DirectVelocityController::stop() noexcept
{
  return velocity(0); // TODO zeroize damper?
}

AngularVelocity DirectVelocityController::velocity() const noexcept
{
  return motor.velocity();
}

RotationalDamping DirectVelocityController::damper() const noexcept
{
  return motor.damper();
}



