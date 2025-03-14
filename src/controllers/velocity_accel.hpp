#ifndef VELOCITY_ACCEL_CONTROLLER_HPP
#define VELOCITY_ACCEL_CONTROLLER_HPP

#include "basic.hpp"
#include "sub.hpp"

namespace kot_motor::controller {

using motor::Limits;
using motor::Motor;

using dimensions::AngularAccel;
using dimensions::AngularVelocity;
using dimensions::Frequency;
using dimensions::Hertz;
using dimensions::Hz;
using dimensions::RotationalDamping;
using dimensions::Time;

class VelocityAccelController : public BasicController {
public:
  struct UserLimits {
    std::optional<Limits<AngularAccel>> acceleration;
    std::optional<Limits<Frequency>> frequency;
    std::optional<Limits<AngularVelocity>> velocity;
    std::optional<Limits<RotationalDamping>> damper;
  };

public:
  VelocityAccelController(
    Motor & motor,
    const UserLimits & userLimits = {},
    AngularAccel accel = 1,
    Frequency freq = 10
  ) noexcept;

  Status reset() noexcept override;

  // Control parameters setters
  Status damper(RotationalDamping damp) noexcept;
  Status acceleration(AngularAccel accel) noexcept;
  Status frequency(Frequency freq) noexcept;
  Status velocity(AngularVelocity vel) noexcept;
  Status velocity(AngularVelocity vel, AngularAccel accel, Frequency freq) noexcept;
  Status stop() noexcept;

  // Control parameters getters
  AngularVelocity velocity() const noexcept;
  RotationalDamping damper() const noexcept;
  AngularAccel acceleration() const noexcept;
  Frequency frequency() const noexcept;

protected:
  Status set_velocity(AngularVelocity vel) noexcept;

protected:
  UserLimits userLimits;
  AngularAccel _accel_;
  Frequency _freq_;
};

} // namespace kot_motor::controller

#endif // VELOCITY_ACCEL_CONTROLLER_HPP



