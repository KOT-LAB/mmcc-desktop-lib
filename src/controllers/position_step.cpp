#include "position_step.hpp"
#include <thread>
#include <chrono>

using kot_motor::controller::PositionStepController;
using namespace kot_motor::dimensions;

PositionStepController::PositionStepController(
  Motor & motor,
  const UserLimits & userLimits,
  Radian step,
  Frequency freq
) noexcept
  : BasicController(motor)
  , userLimits(userLimits)
  , _step_(
      userLimits.step.has_value()
        ? limitUnitBy(
            step, userLimits.step.value().min, userLimits.step.value().max
          )
        : step
    ) // TODO looks lousy
  , _freq_(
      userLimits.frequency.has_value() ? limitUnitBy(
                                           freq,
                                           userLimits.frequency.value().min,
                                           userLimits.frequency.value().max
                                         )
                                       : freq
    ) // TODO looks lousy
{ }

PositionStepController::Status PositionStepController::reset() noexcept
{
  motor.position(0);
  motor.stiffness(0);
  motor.damper(0);
  _step_ = 0;
  _freq_ = 0; // TODO normal ?

  return Status::SUCCESS;
}

/*********************** Control parameters setters ************************/

PositionStepController::Status PositionStepController::step(Radian step) noexcept
{
  if (userLimits.step.has_value()) {
    auto && limits = userLimits.step.value();
    step = dimensions::limitUnitBy(step, limits.min, limits.max);
  }
  _step_ = step;
  return Status::SUCCESS;
}

PositionStepController::Status PositionStepController::frequency(Frequency freq) noexcept
{
  if (userLimits.frequency.has_value()) {
    auto && limits = userLimits.frequency.value();
    freq = dimensions::limitUnitBy(freq, limits.min, limits.max);
  }
  _freq_ = freq;
  return Status::SUCCESS;
}

PositionStepController::Status PositionStepController::position(Radian pos) noexcept
{
  return position(pos, _step_, _freq_);
}

PositionStepController::Status
  PositionStepController::position(Radian pos, Radian step, Frequency freq) noexcept
{
  Radian actPos = motor.position();
  if (motor.state() == Motor::MotorState::MOTOR_MODE_NOT_ACTIVE) {
    return Status::MOTOR_NOT_SWITCHED_ON;
  } else if (pos == actPos) {
    return Status::ALREADY_DONE;
  } else if (_step_ == Radian(0) or freq == Frequency(0)) {
    return Status::FAIL;
  }

  /************************ limiting *************************/
  auto && limits =
    userLimits.position.value_or(motor.motorInfo().motorHwLimits.position);
  Radian desPos = dimensions::limitUnitBy(pos, limits.min, limits.max);
  if (userLimits.step.has_value()) {
    auto && limits = userLimits.step.value();
    step = dimensions::limitUnitBy(step, limits.min, limits.max);
  }
  if (userLimits.frequency.has_value()) {
    auto && limits = userLimits.frequency.value();
    freq = dimensions::limitUnitBy(freq, limits.min, limits.max);
  }

  /************************ movement *************************/
  const float k_error = 1.1;
  const Time one_sending_time = 1.0 / 1000000.0 * k_error;
  Radian acceptable_pos_error = deg_to_rad(0.01);

  Radian delta = desPos - actPos;
  Direction direction = float(delta) > 0 ? Direction::RIGHT : Direction::LEFT;
  uint32_t stepsN = std::abs(float(delta) / float(step));
  Radian pos_error = float(delta) - float(step) * stepsN;
  Time total_t = (stepsN / float(freq)) - (stepsN * float(one_sending_time));
  uint32_t delay_between_sendings_us = 1000000 * int(total_t) / stepsN;

  BasicTransport::Status status = BasicTransport::Status::SUCCESS;
  if (direction == Direction::RIGHT) {
    for (uint32_t stepN = 0; stepN < stepsN; stepN++) {
      motor.position(actPos += step);
      status = motor.sendToMotor();
      if (status == BasicTransport::Status::FAIL) {
        break;
      }
      std::this_thread::sleep_for(
        std::chrono::microseconds(delay_between_sendings_us)
      );
    }
    if (pos_error > acceptable_pos_error and status != BasicTransport::Status::FAIL) {
      motor.position(actPos += pos_error);
      status = motor.sendToMotor();
    }
  } else {
    for (uint32_t stepN = 0; stepN < stepsN; stepN++) {
      motor.position(actPos -= step);
      status = motor.sendToMotor();
      if (status == BasicTransport::Status::FAIL) {
        break;
      }
      std::this_thread::sleep_for(
        std::chrono::microseconds(delay_between_sendings_us)
      );
    }
    if (pos_error > acceptable_pos_error and status != BasicTransport::Status::FAIL) {
      motor.position(actPos -= pos_error);
      status = motor.sendToMotor();
    }
  }

  switch (status) {
    case BasicTransport::Status::SUCCESS:
      return Status::SUCCESS;
    default:
      return Status::FAIL;
  }
}

PositionStepController::Status
  PositionStepController::stiffeness(RotationalStiffness stiff) noexcept
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

PositionStepController::Status
  PositionStepController::damper(RotationalDamping damp) noexcept
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

Radian PositionStepController::step() const noexcept
{
  return _step_;
}

Frequency PositionStepController::frequency() const noexcept
{
  return _freq_;
}

Radian PositionStepController::position() const noexcept
{
  return motor.position();
}

RotationalStiffness PositionStepController::stiffeness() const noexcept
{
  return motor.stiffness();
}

RotationalDamping PositionStepController::damper() const noexcept
{
  return motor.damper();
}




