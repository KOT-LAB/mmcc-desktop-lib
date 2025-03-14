#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "motor.hpp"
#include "dimensions/dimensions.hpp"

namespace kot_motor::config {

using namespace kot_motor;
using namespace kot_motor::motor;

using dimensions::AngularVelocity;
using dimensions::deg_to_rad;
using dimensions::Radian;
using dimensions::RotationalDamping;
using dimensions::RotationalStiffness;
using dimensions::Torque;

inline Motor::MotorInfo default_motor{

  Motor::MotorSpecification{"default", "default", 500, 24, 0.12f, 5, 16, 50},

  Motor::MotorLimits{Limits<Radian>{-12.5f, 12.5f}, Limits<AngularVelocity>{-50, 50}, Limits<Torque>{-16, 16}, Limits<RotationalStiffness>{0, 500}, Limits<RotationalDamping>{0.0f, 5.0f}}
};

inline Motor::MotorInfo cubemars_ak7010{

  Motor::MotorSpecification{
                            "Cubemars", "AK70-10", 500, 24, 0.12f, 8.8f, 24.5f, 50
  },

  Motor::MotorLimits{Limits<Radian>{-12.5f, 12.5f}, Limits<AngularVelocity>{-50, 50}, Limits<Torque>{-24.5f, 24.5f}, Limits<RotationalStiffness>{0, 500}, Limits<RotationalDamping>{0.0f, 5.0f}}
};

} // namespace kot_motor::config

#endif // CONFIG_HPP




