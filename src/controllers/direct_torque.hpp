#ifndef DIRECT_TORQUE_CONTROLLER
#define DIRECT_TORQUE_CONTROLLER

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
using dimensions::RotationalStiffness;
using dimensions::Time;
using dimensions::Torque;

class DirectTorqueController : public BasicController {
public:
  struct UserLimits {
    std::optional<Limits<Torque>> torque;
    std::optional<Limits<RotationalStiffness>> stiffeness;
    std::optional<Limits<RotationalDamping>> damper;
  };

public:
  DirectTorqueController(
    Motor & motor, const UserLimits & userLimits = {}
  ) noexcept;

  Status reset() noexcept override;

  // Control parameters setters
  Status torque(Torque torque) noexcept;
  Status stiffeness(RotationalStiffness stiff) noexcept;
  Status damper(RotationalDamping damp) noexcept;

  // Control parameters getters
  Torque torque() const noexcept;
  RotationalStiffness stiffeness() const noexcept;
  RotationalDamping damper() const noexcept;

protected:
  UserLimits userLimits;
};

} // namespace kot_motor::controller

#endif // DIRECT_TORQUE_CONTROLLER



