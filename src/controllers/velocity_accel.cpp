#include "velocity_accel.hpp"
#include <string>
#include <thread>
#include <chrono>

using kot_motor::controller::VelocityAccelController;
using namespace kot_motor::dimensions;

VelocityAccelController::VelocityAccelController(
  Motor & motor, const UserLimits & userLimits, AngularAccel accel, Frequency freq
) noexcept
  : BasicController(motor)
  , userLimits(userLimits)
  , _accel_(
      userLimits.acceleration.has_value()
        ? limitUnitBy(accel, userLimits.acceleration.value().min, userLimits.acceleration.value().max)
        : accel
    ) // TODO looks lousy
  , _freq_(
      userLimits.frequency.has_value()
        ? limitUnitBy(freq, userLimits.frequency.value().min, userLimits.frequency.value().max)
        : freq
    ) // TODO looks lousy
{ }

VelocityAccelController::Status VelocityAccelController::reset() noexcept
{
  motor.velocity(0);
  motor.damper(0);
  _accel_ = 0;
  _freq_ = 0; // TODO normal ?

  BasicTransport::Status canStatus = motor.sendToMotor();

  switch (canStatus) {
    case BasicTransport::Status::SUCCESS:
      return Status::SUCCESS;
    default:
      return Status::FAIL;
  }
}

/*********************** Control parameters setters ************************/

VelocityAccelController::Status VelocityAccelController::VelocityAccelController::stop() noexcept
{
  return set_velocity(0);
}

VelocityAccelController::Status VelocityAccelController::damper(RotationalDamping damp) noexcept
{
  if (motor.state() == Motor::MotorState::MOTOR_MODE_NOT_ACTIVE) {
    return Status::MOTOR_NOT_SWITCHED_ON;
  } else if (damp == motor.damper()) {
    return Status::ALREADY_DONE;
  }

  auto limits = userLimits.damper.value_or(motor.motorInfo().motorHwLimits.damper);
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

VelocityAccelController::Status VelocityAccelController::acceleration(AngularAccel accel) noexcept
{
  if (userLimits.acceleration.has_value()) {
    auto limits = userLimits.acceleration.value();
    accel = dimensions::limitUnitBy(accel, limits.min, limits.max);
  }
  _accel_ = accel;
  return Status::SUCCESS;
}

VelocityAccelController::Status VelocityAccelController::frequency(Frequency freq) noexcept
{
  // always check for freq validity
  if (!(freq > Frequency(0))) {
    return Status::FAIL;
  }
  if (userLimits.frequency.has_value()) {
    auto limits = userLimits.frequency.value();
    freq = dimensions::limitUnitBy(freq, limits.min, limits.max);
  }
  _freq_ = freq;
  return Status::SUCCESS;
}

VelocityAccelController::Status VelocityAccelController::velocity(AngularVelocity vel) noexcept
{
  return velocity(vel, _accel_, _freq_);
}

VelocityAccelController::Status
  VelocityAccelController::velocity(AngularVelocity vel, AngularAccel accel, Frequency freq) noexcept
{
  auto v0 = motor.velocity();

  if (motor.state() == Motor::MotorState::MOTOR_MODE_NOT_ACTIVE) {
    return Status::MOTOR_NOT_SWITCHED_ON;
  } else if (vel == v0) {
    return Status::ALREADY_DONE;
  } else if (freq == Frequency(0)) {
    return Status::FAIL;
  }

  auto && vel_limits = userLimits.velocity.value_or(
    motor.motorInfo().motorHwLimits.velocity
  );
  AngularVelocity v1 = dimensions::limitUnitBy(
    vel, vel_limits.min, vel_limits.max
  );

  if (userLimits.acceleration.has_value()) {
    auto && accel_limits = userLimits.acceleration.value();
    accel = dimensions::limitUnitBy(
      accel, accel_limits.min, accel_limits.max
    );
  }
  if (userLimits.frequency.has_value()) {
    auto && freq_limits = userLimits.frequency.value();
    freq = dimensions::limitUnitBy(
      freq, freq_limits.min, freq_limits.max
    );
  }

  auto get_vel = [v0, accel](Time t) -> AngularVelocity {
    return v0 + accel * t;
  };

  // based on the top mcp2515 frequency
  const float k_error = 1.1;
  const Time one_sending_time = 0.0002 * k_error;

  Time delta_t = (v1 - v0) / accel; // in sec
  uint32_t sendings_n = float(delta_t) * float(freq);
  uint32_t delay_between_sendings_us =
    1000000 * (float(delta_t) / sendings_n) -
    (float(one_sending_time) * sendings_n);

  Status status = Status::SUCCESS;
  for (uint32_t i = 1; i <= sendings_n; i++) {
    Time t = i * float(delta_t) / sendings_n; // sec
    AngularVelocity v = get_vel(t);
    status = set_velocity(v);

    if (status == Status::FAIL) {
      break;
    }

    std::this_thread::sleep_for(
      std::chrono::microseconds(delay_between_sendings_us)
    );
  }

  return status;
}

/*********************** **Direct velocity setter **************************/

VelocityAccelController::Status VelocityAccelController::set_velocity(AngularVelocity vel) noexcept
{
  motor.velocity(vel);
  BasicTransport::Status canStatus = motor.sendToMotor();

  switch (canStatus) {
    case BasicTransport::Status::SUCCESS:
      return Status::SUCCESS;
    default:
      return Status::FAIL;
  }
}

/*********************** Control parameters getters ************************/

AngularVelocity VelocityAccelController::velocity() const noexcept
{
  return motor.velocity();
}

RotationalDamping VelocityAccelController::damper() const noexcept
{
  return motor.damper();
}

AngularAccel VelocityAccelController::acceleration() const noexcept
{
  return _accel_;
}

Frequency VelocityAccelController::frequency() const noexcept
{
  return _freq_;
}



