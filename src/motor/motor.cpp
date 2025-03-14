#include "motor.hpp"
#include "basic_transport.hpp"

using namespace kot_motor::motor;
using kot_motor::motor::Motor;
using kot_motor::transport::BasicTransport;

Motor::Motor(
  BasicTransport & bus, uint8_t canId, uint8_t masterCanId, const MotorInfo & config
) noexcept
  : bus(bus)
  , canId(canId)
  , masterCanId(masterCanId)
  , config(config)
  , inputParams()
  , outputParams()
{ }

Motor::Motor(Motor && other) = default;
Motor::~Motor() = default;

/***************************** State Control *******************************/

BasicTransport::Status Motor::enterMotorMode()
{
  CanFrameBuff buff = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  BasicTransport::CanFrame canFrame(
    canId, masterCanId, buff.data(), buff.size()
  );

  BasicTransport::Status status = sendCmd(canFrame);

  if (status == BasicTransport::Status::SUCCESS) {
    motorState = MotorState::MOTOR_MODE_ACTIVE;
  }

  return status;
}

BasicTransport::Status Motor::exitMotorMode()
{
  CanFrameBuff buff = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  BasicTransport::CanFrame canFrame(
    canId, masterCanId, buff.data(), buff.size()
  );

  BasicTransport::Status status = sendCmd(canFrame);

  if (status == BasicTransport::Status::SUCCESS) {
    motorState = MotorState::MOTOR_MODE_NOT_ACTIVE;
  }

  return status;
}

BasicTransport::Status Motor::setProgramZero()
{
  CanFrameBuff buff = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
  BasicTransport::CanFrame canFrame(
    canId, masterCanId, buff.data(), buff.size()
  );

  BasicTransport::Status status = sendCmd(canFrame);

  return status;
}

BasicTransport::Status Motor::resetParameters()
{
  inputParams.position = 0.0f;
  inputParams.velocity = 0.0f;
  inputParams.torque = 0.0f;
  inputParams.stiffness = 0.0f;
  inputParams.damper = 0.0f;

  outputParams.position = 0.0f;
  outputParams.velocity = 0.0f;
  outputParams.torque = 0.0f;

  BasicTransport::Status status = sendToMotor();

  return status;
}

/*************************** Can communication *****************************/

BasicTransport::Status Motor::sendToMotor()
{
  auto cmd = packCmd(inputParams);
  auto status = sendCmd(cmd);
  return status;
}

BasicTransport::Status Motor::getActualParameters()
{
  auto replay = getReply();
  if (replay.has_value()) {
    outputParams = unpackReplay(replay.value());
    return BasicTransport::Status::SUCCESS;
  } else {
    return BasicTransport::Status::FAIL;
  }
}

/************************ InputParameters setters **************************/

void Motor::position(Radian pos)
{
  setParameterHelper(pos, inputParams.position, config.motorHwLimits.position);
}

void Motor::velocity(AngularVelocity vel)
{
  setParameterHelper(vel, inputParams.velocity, config.motorHwLimits.velocity);
}

void Motor::torque(Torque torq)
{
  setParameterHelper(torq, inputParams.torque, config.motorHwLimits.torque);
}

void Motor::stiffness(RotationalStiffness stiff)
{
  setParameterHelper(
    stiff, inputParams.stiffness, config.motorHwLimits.stiffness
  );
}

void Motor::damper(RotationalDamping damp)
{
  setParameterHelper(damp, inputParams.damper, config.motorHwLimits.damper);
}

/*********************** Motor information getters *************************/

uint8_t Motor::canID() const
{
  return canId;
}

uint8_t Motor::masterCanID() const
{
  return masterCanId;
}

const BasicTransport & Motor::transport() const
{
  return bus;
}

const Motor::MotorInfo & Motor::motorInfo() const
{
  return config;
}

/****************************** State getters *****************************/

Motor::MotorState Motor::state() const
{
  return motorState;
}

const Motor::InputParameters & Motor::inputParameters() const
{
  return inputParams;
}

const Motor::OutputParameters & Motor::outputParameters() const
{
  return outputParams;
}

/************************ InputParameters getters **************************/

Radian Motor::position() const
{
  return inputParams.position;
}

AngularVelocity Motor::velocity() const
{
  return inputParams.velocity;
}

Torque Motor::torque() const
{
  return inputParams.torque;
}

RotationalStiffness Motor::stiffness() const
{
  return inputParams.stiffness;
}

RotationalDamping Motor::damper() const
{
  return inputParams.damper;
}

/************************ OutputParamters getters *************************/

Radian Motor::actualPosition() const
{
  return outputParams.position;
}

AngularVelocity Motor::actualVelocity() const
{
  return outputParams.velocity;
}

Torque Motor::actualTorque() const
{
  return outputParams.torque;
}

/***************** Packing/unpacking, sending/receivring ******************/

BasicTransport::CanFrame Motor::packCmd(const InputParameters & inParams)
{
  /*
   * CAN Command Packet Structure
   *
   * 16 bit position command, between -4*pi and 4*pi
   * 12 bit velocity command, between -30 and + 30 rad/s
   * 12 bit kp, between 0 and 500 N-m/rad
   * 12 bit kd, between 0 and 100 N-m*s/rad
   * 12 bit feed forward torque, between -18 and 18 N-m
   * CAN Packet is 8 8-bit words
   * Formatted as follows.  For each quantity, bit 0 is LSB
   * 0: [position[15-8]]
   * 1: [position[7-0]]
   * 2: [velocity[11-4]]
   * 3: [velocity[3-0], kp[11-8]]
   * 4: [kp[7-0]]
   * 5: [kd[11-4]]
   * 6: [kd[3-0], torque[11-8]]
   * 7: [torque[7-0]]
   */

  // constrain parameters ///
  Radian p_des = dimensions::limitUnitBy(
    inputParams.position,
    config.motorHwLimits.position.min,
    config.motorHwLimits.position.max
  );
  AngularVelocity v_des = dimensions::limitUnitBy(
    inputParams.velocity,
    config.motorHwLimits.velocity.min,
    config.motorHwLimits.velocity.max
  );
  Torque t_ff = dimensions::limitUnitBy(
    inputParams.torque,
    config.motorHwLimits.torque.min,
    config.motorHwLimits.torque.max
  );
  RotationalStiffness kp = dimensions::limitUnitBy(
    inputParams.stiffness,
    config.motorHwLimits.stiffness.min,
    config.motorHwLimits.stiffness.max
  );
  RotationalDamping kd = dimensions::limitUnitBy(
    inputParams.damper,
    config.motorHwLimits.damper.min,
    config.motorHwLimits.damper.max
  );

  /// convert floats to uint32_ts ///
  uint32_t p_int = floatToUint(
    static_cast<float>(p_des),
    static_cast<float>(config.motorHwLimits.position.min),
    static_cast<float>(config.motorHwLimits.position.max),
    16
  );
  uint32_t v_int = floatToUint(
    static_cast<float>(v_des),
    static_cast<float>(config.motorHwLimits.velocity.min),
    static_cast<float>(config.motorHwLimits.velocity.max),
    12
  );
  uint32_t t_int = floatToUint(
    static_cast<float>(t_ff),
    static_cast<float>(config.motorHwLimits.torque.min),
    static_cast<float>(config.motorHwLimits.torque.max),
    12
  );
  uint32_t kp_int = floatToUint(
    static_cast<float>(kp),
    static_cast<float>(config.motorHwLimits.stiffness.min),
    static_cast<float>(config.motorHwLimits.stiffness.max),
    12
  );
  uint32_t kd_int = floatToUint(
    static_cast<float>(kd),
    static_cast<float>(config.motorHwLimits.damper.min),
    static_cast<float>(config.motorHwLimits.damper.max),
    12
  );

  // TODO get an feedback here ?

  CanFrameBuff buff;
  buff[0] = p_int >> 8;
  buff[1] = p_int & 0xFF;
  buff[2] = v_int >> 4;
  buff[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  buff[4] = kp_int & 0xFF;
  buff[5] = kd_int >> 4;
  buff[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  buff[7] = t_int & 0xFF;

  return BasicTransport::CanFrame{canId, masterCanId, buff.data(), buff.size()};
}

BasicTransport::Status Motor::sendCmd(const BasicTransport::CanFrame & canFrame)
{
  BasicTransport::Status status = bus.write(canFrame);
  return status;
}

std::optional<BasicTransport::CanFrame> Motor::getReply()
{
  // if (bus.available())
  // return {};

  // TODO how readMsgBuf() function works ????

  // reading all up to the most actual data
  // uint8_t len = 0;
  // unsigned long sender_id = 0; // why this does work?
  // uint64_t sender_id = 0; // and this does not ?????
  BasicTransport::CanFrame canFrame;
  // while (bus.available()) {
  //  bus.read(&sender_id, &len, buff.data());
  //}
  return canFrame;
}

Motor::OutputParameters Motor::unpackReplay(const BasicTransport::CanFrame & canFrame)
{
  /*
   * CAN Reply Packet Structure:
   *
   * 16 bit position, between -4*pi and 4*pi
   * 12 bit velocity, between -30 and + 30 rad/s
   * 12 bit current, between -40 and 40;
   * CAN Packet is 5 8-bit words
   * Formatted as follows.  For each quantity, bit 0 is LSB
   * 0: [position[15-8]]
   * 1: [position[7-0]]
   * 2: [velocity[11-4]]
   * 3: [velocity[3-0], current[11-8]]
   * 4: [current[7-0]]
   * float p_des = constrain(this->position, this->positionLimits[0],
   * this->positionLimits[1]);
   */

  // uint32_t id = buff[0]; // TODO what's this?
  uint32_t p_int = (canFrame.data[1] << 8) | canFrame.data[2];
  uint32_t v_int = (canFrame.data[3] << 4) | (canFrame.data[4] >> 4);
  uint32_t i_int = ((canFrame.data[4] & 0xF) << 8) | canFrame.data[5];

  OutputParameters feedback;

  feedback.position = uintToFloat(
    p_int,
    static_cast<float>(config.motorHwLimits.position.min),
    static_cast<float>(config.motorHwLimits.position.max),
    16
  );
  feedback.velocity = uintToFloat(
    v_int,
    static_cast<float>(config.motorHwLimits.velocity.min),
    static_cast<float>(config.motorHwLimits.velocity.max),
    12
  );
  feedback.torque = uintToFloat(
    i_int,
    static_cast<float>(config.motorHwLimits.torque.min),
    -static_cast<float>(config.motorHwLimits.torque.max),
    12
  );

  return feedback;
}

constexpr uint32_t Motor::floatToUint(float x, float x_min, float x_max, uint8_t bits)
{
  /* Converts a float to an unsigned int, given range and number of bits */
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;

  if (bits == 12) {
    pgg = (unsigned int)((x - offset) * 4095.0 / span);
  } else if (bits == 16) {
    pgg = (unsigned int)((x - offset) * 65535.0 / span);
  }

  return pgg;
}

constexpr float Motor::uintToFloat(uint32_t x_int, float x_min, float x_max, uint8_t bits)
{
  /* converts uint32_t to float, given range and number of bits */
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;

  if (bits == 12) {
    pgg = ((float)x_int) * span / 4095.0 + offset;
  } else if (bits == 16) {
    pgg = ((float)x_int) * span / 65535.0 + offset;
  }

  return pgg;
}



