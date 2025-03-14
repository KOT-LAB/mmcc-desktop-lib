#include "direct_torque.hpp"

using kot_motor::controller::DirectTorqueController;
using namespace kot_motor::dimensions;

DirectTorqueController::DirectTorqueController(
  Motor & motor, const UserLimits & userLimits
) noexcept
  : BasicController(motor), userLimits(userLimits)
{ }

DirectTorqueController::Status DirectTorqueController::reset() noexcept
{
  motor.torque(0);
  return Status::SUCCESS;
}

/*********************** Control parameters setters ************************/

DirectTorqueController::Status DirectTorqueController::torque(Torque torq) noexcept
{
  if (motor.state() == Motor::MotorState::MOTOR_MODE_NOT_ACTIVE) {
    return Status::MOTOR_NOT_SWITCHED_ON;
  } else if (torq == motor.torque()) {
    return Status::ALREADY_DONE;
  }

  auto && limits =
    userLimits.torque.value_or(motor.motorInfo().motorHwLimits.torque);

  torq = dimensions::limitUnitBy(torq, limits.min, limits.max);

  motor.torque(torq);
  BasicTransport::Status canStatus = motor.sendToMotor();

  switch (canStatus) {
    case BasicTransport::Status::SUCCESS:
      return Status::SUCCESS;
    default:
      return Status::FAIL;
  }
}

DirectTorqueController::Status
  DirectTorqueController::stiffeness(RotationalStiffness stiff) noexcept
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

DirectTorqueController::Status
  DirectTorqueController::damper(RotationalDamping damp) noexcept
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

Torque DirectTorqueController::torque() const noexcept
{
  return motor.torque();
}

RotationalStiffness DirectTorqueController::stiffeness() const noexcept
{
  return motor.stiffness();
}

RotationalDamping DirectTorqueController::damper() const noexcept
{
  return motor.damper();
}



