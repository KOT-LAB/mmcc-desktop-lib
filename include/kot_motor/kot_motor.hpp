#ifndef KOT_MOTOR_HPP
#define KOT_MOTOR_HPP

#include "src/motor/configs.hpp"
#include "src/motor/motor.hpp"
#include "src/controllers/direct_position.hpp"
#include "src/controllers/direct_velocity.hpp"
#include "src/controllers/direct_torque.hpp"
#include "src/controllers/velocity_accel.hpp"
#include "src/controllers/position_step.hpp"
#include "src/transport/socketcan_transport.hpp"

namespace kot_motor {

using kot_motor::motor::Motor;
using kot_motor::transport::SocketCanTransport;
using namespace kot_motor::dimensions;
using namespace kot_motor::controller;

} // namespace kot_motor

#endif // KOT_MOTOR_HPP




