#ifndef POSITION_STEP_CONTROLLER_HPP
#define POSITION_STEP_CONTROLLER_HPP

#include "basic.hpp"
#include <optional>
#include "sub.hpp"

namespace kot_motor::controller {

using motor::Limits;
using motor::Motor;

using dimensions::deg_to_rad;
using dimensions::Frequency;
using dimensions::rad_to_deg;
using dimensions::Radian;
using dimensions::RotationalDamping;
using dimensions::RotationalStiffness;
using dimensions::Time;

class PositionStepController : public BasicController {
public:
  struct UserLimits {
    std::optional<Limits<Radian>> step;
    std::optional<Limits<Frequency>> frequency;
    std::optional<Limits<Radian>> position;
    std::optional<Limits<RotationalStiffness>> stiffeness;
    std::optional<Limits<RotationalDamping>> damper;
  };

protected:
  enum class Direction {
    RIGHT,
    LEFT
  };

public:
  PositionStepController(
    Motor & motor,
    const UserLimits & userLimits = {},
    Radian step = deg_to_rad(0.1),
    Frequency freq = FREQ_1KHz
  ) noexcept;

  Status reset() noexcept override;

  // Control parameters setters
  Status frequency(Frequency) noexcept;
  Status step(Radian step) noexcept;
  Status position(Radian pos) noexcept;
  Status position(Radian pos, Radian step, Frequency freq = FREQ_1KHz) noexcept;
  Status stiffeness(RotationalStiffness stiff) noexcept;
  Status damper(RotationalDamping damp) noexcept;

  // Control parameters getters
  Radian step() const noexcept;
  Frequency frequency() const noexcept;
  Radian position() const noexcept;
  RotationalStiffness stiffeness() const noexcept;
  RotationalDamping damper() const noexcept;

protected:
  UserLimits userLimits;
  Radian _step_;
  Frequency _freq_;
};

} // namespace kot_motor::controller

#endif // POSITION_STEP_CONTROLLER_HPP




