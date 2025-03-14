#ifndef DIRECT_POSITION_CONTROLLER_HPP
#define DIRECT_POSITION_CONTROLLER_HPP

#include "basic.hpp"

namespace kot_motor::controller {

using motor::Limits;
using motor::Motor;

using dimensions::Radian;
using dimensions::RotationalDamping;
using dimensions::RotationalStiffness;

class DirectPositionController : public BasicController {
public:
  struct UserLimits {
    std::optional<Limits<Radian>> position;
    std::optional<Limits<RotationalStiffness>> stiffeness;
    std::optional<Limits<RotationalDamping>> damper;
  };

public:
  DirectPositionController(
    Motor & motor, const UserLimits & userLimits = {}
  ) noexcept;

  // Resets all the controll parameters of controller
  Status reset() noexcept override;

  // Control parameters setters
  Status position(Radian pos) noexcept;
  Status stiffeness(RotationalStiffness stiff) noexcept;
  Status damper(RotationalDamping damp) noexcept;

  // Control parameters getters
  Radian position() const noexcept;
  RotationalStiffness stiffeness() const noexcept;
  RotationalDamping damper() const noexcept;

protected:
  UserLimits userLimits;
};

} // namespace kot_motor::controller

#endif // DIRECT_VELOCITY_CONTROLLER_HPP




