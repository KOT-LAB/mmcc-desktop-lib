#ifndef MOTOR_HPP
#define MOTOR_HPP


#include <array>
#include <optional>
#include "dimensions/dimensions.hpp"
#include "transport/basic_transport.hpp"
#include <stdint.h>
#include <string>

namespace kot_motor::motor {

using dimensions::AngularVelocity;
using dimensions::Degree;
using dimensions::Radian;
using dimensions::RotationalDamping;
using dimensions::RotationalStiffness;
using dimensions::Torque;
using dimensions::Voltage;
using dimensions::Weight;
using transport::BasicTransport;
using CanFrameBuff = std::array<uint8_t, 8>;

template <typename Unit>
struct Limits {
  Unit min;
  Unit max;
};

class Motor {
public:
  enum class MotorState : uint8_t {
    MOTOR_MODE_ACTIVE,
    MOTOR_MODE_NOT_ACTIVE
  };

  struct MotorSpecification {
    std::string manufacturer;
    std::string model;

    Weight weight;
    Voltage voltage;

    Degree precision;
    Torque ratedTorque;
    Torque peakTorque;
    AngularVelocity maxSpeed;
  };

  struct MotorLimits {
    Limits<Radian> position;
    Limits<AngularVelocity> velocity;
    Limits<Torque> torque;
    Limits<RotationalStiffness> stiffness;
    Limits<RotationalDamping> damper;
  };

  struct MotorInfo {
    MotorSpecification motorSpecification;
    MotorLimits motorHwLimits;
  };

private:
  struct InputParameters {
    Radian position;
    AngularVelocity velocity;
    Torque torque;
    RotationalStiffness stiffness;
    RotationalDamping damper;
  };

  struct OutputParameters {
    Radian position;
    AngularVelocity velocity;
    Torque torque;
  };

private:
  BasicTransport & bus;
  uint8_t canId;
  uint8_t masterCanId;

  MotorInfo config;
  InputParameters inputParams;
  OutputParameters outputParams;

  MotorState motorState = MotorState::MOTOR_MODE_NOT_ACTIVE;

public:
  Motor(BasicTransport & bus, uint8_t canId, uint8_t masterCanId, const MotorInfo & config) noexcept;
  Motor(Motor && other);
  ~Motor();

  // State control
  BasicTransport::Status enterMotorMode();
  BasicTransport::Status exitMotorMode();
  BasicTransport::Status setProgramZero();
  BasicTransport::Status resetParameters();

  // Can communication
  BasicTransport::Status sendToMotor();
  BasicTransport::Status getActualParameters();

  // InputParameters setters
  void position(Radian pos);
  void velocity(AngularVelocity vel);
  void torque(Torque torq);
  void stiffness(RotationalStiffness stiff);
  void damper(RotationalDamping damp);

  // Motor information getters
  uint8_t canID() const;
  uint8_t masterCanID() const;
  const BasicTransport & transport() const;
  const MotorInfo & motorInfo() const;

  // State getters
  MotorState state() const;
  const InputParameters & inputParameters() const;
  const OutputParameters & outputParameters() const;

  // InputParameters getters
  Radian position() const;
  AngularVelocity velocity() const;
  Torque torque() const;
  RotationalStiffness stiffness() const;
  RotationalDamping damper() const;

  // OutputParameters getters
  Radian actualPosition() const;
  AngularVelocity actualVelocity() const;
  Torque actualTorque() const;

private:
  // Packing/unpacking, sending/receivring
  BasicTransport::CanFrame packCmd(const InputParameters & inParams);
  BasicTransport::Status sendCmd(const BasicTransport::CanFrame & canFrame);

  OutputParameters unpackReplay(const BasicTransport::CanFrame & canFrame);
  std::optional<BasicTransport::CanFrame> getReply();

  constexpr uint32_t floatToUint(float x, float x_min, float x_max, uint8_t bits);
  constexpr float uintToFloat(uint32_t x_int, float x_min, float x_max, uint8_t bits);

  template <typename Param, typename T, typename Unit>
  constexpr void setParameterHelper(
    Param val,
    dimensions::Unit<T> & inputParamsVal,
    const Limits<Unit> & limits
  );
};

template <typename Param, typename T, typename Unit>
constexpr void Motor::setParameterHelper(
  Param val, dimensions::Unit<T> & inputParamsVal, const Limits<Unit> & limits
)
{
  if (val == inputParamsVal) {
    return;
  }
  inputParamsVal = dimensions::limitUnitBy(val, limits.min, limits.max);
}

} // namespace kot_motor::motor

#endif // MOTOR_HPP




