#ifndef DIRECT_VELOCITY_CONTROLLER_HPP
#define DIRECT_VELOCITY_CONTROLLER_HPP

#include "basic.hpp"

namespace kot_motor::controller {

using motor::Limits;
using motor::Motor;

using dimensions::AngularVelocity;
using dimensions::RotationalDamping;

class DirectVelocityController : public BasicController {
public:
  struct UserLimits {
    std::optional<Limits<AngularVelocity>> velocity;
    std::optional<Limits<RotationalDamping>> damper;
  };

  DirectVelocityController(
    Motor & motor, const UserLimits & userLimits = {}
  ) noexcept;

  Status reset() noexcept override;

  // Control parameters setters
  Status velocity(AngularVelocity vel) noexcept;
  Status damper(RotationalDamping damp) noexcept;
  Status stop() noexcept;

  // Control parameters getters
  AngularVelocity velocity() const noexcept;
  RotationalDamping damper() const noexcept;

protected:
  UserLimits userLimits;
};

} // namespace kot_motor::controller

#endif // DIRECT_VELOCITY_CONTROLLER_HPP




