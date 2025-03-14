#ifndef SUB_HPP
#define SUB_HPP

#include "motor/motor.hpp"

namespace kot_motor::controller {

enum class Frequencys : uint32_t {
  FREQ_1DHz = (uint32_t)10,
  FREQ_1HHz = (uint32_t)10e2,
  FREQ_1KHz = (uint32_t)10e3,
  FREQ_1MHz = (uint32_t)10e6
};

#define FREQ_1DHz (Frequency(10))
#define FREQ_1HHz (Frequency(10e2))
#define FREQ_1KHz (Frequency(10e3))
#define FREQ_1MHz (Frequency(10e6))

} // namespace kot_motor::controller

#endif // SUB_HPP



