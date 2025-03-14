# MIT Mini Cheetah Controller Library

## Overview

This repository provides a **high-level, lightweight, and safe C++ library** for interfacing with the **MIT Mini Cheetah Controller** over a CAN bus. Designed for **bio-inspired robotics**, this library enables **safe, precise, and extendable motor control** while ensuring **compile-time unit safety** and **hardware protection**.

### Key Features & Advantages

- **Modular & Extensible** – Users can implement custom control modes and transport mechanisms.
- **Easily Configurable** – Users can set up their own motor configurations or use predefined ones from `configs.hpp`.
- **High-Level Abstraction** – Provides user-friendly and safe controllers for position, torque, velocity, and step-based motion.
- **Transport Abstraction** – Supports different communication methods, including SocketCAN.
- **Safe & Robust** – Enforces motor limits, ensuring hardware protection.
- **Type-Safe Computations** – Uses a custom `dimensions` template library for compile-time unit safety and self-documenting code.

---

## Directory Structure

```
.
├── examples                 # Contains example projects demonstrating how to use the library
├── include                  # Header files for the library
│   └── kot_motor            # Main library include directory
└── src                      # Library source files
    ├── motor                # Core motor class implementation
    ├── controllers          # High-level motor control classes (e.g., Position, Velocity, Torque controllers)
    ├── transport            # Communication transport layer (e.g., CAN communication)
    └── dimensions           # The custom template-based dimensions library for type safety
```

---

## Library Structure

### Motor Class

The `Motor` class is the **core low-level interface** for interacting with the controller. It handles motor state transitions, parameter settings, and CAN communication.

```cpp
Motor motor(canBus, motorID, masterID, motorConfig);
motor.enterMotorMode();
motor.position(1.0_rad);
motor.sendToMotor();
```

### Controller Classes

Instead of working with the `Motor` class directly, users interact with **high-level controllers**, which are safe and easy to use. The library provides:

- **`DirectPositionController`** – Controls position.
- **`DirectVelocityController`** – Controls velocity.
- **`DirectTorqueController`** – Controls torque.
- **`PositionStepController`** – Moves in discrete steps.
- **`VelocityAccelController`** – Controls velocity with acceleration constraints.

Example:

```cpp
DirectPositionController posCtrl(motor);
posCtrl.switchOn();
posCtrl.position(2.0_rad);
```

### Transport Layer (Communication Abstraction)

The `BasicTransport` class abstracts the **communication with the motor**, allowing users to choose different transport methods.

#### **Supported CAN Interfaces:**

- **Linux Desktop + [CandleLight USB-CAN Adapter](https://linux-automation.com/en/products/candlelight.html)**
- **Raspberry Pi / Nvidia Jetson + [CAN Shield](https://www.waveshare.com/2-ch-can-hat.htm)**

> ⚠️ **Note:** We highly recommend using **galvanic isolation** when connecting to the motor to protect both the controller and the computer from potential damage.

Example using **SocketCAN**:

```cpp
SocketCanTransport transport("can0");
transport.open();
```

### 4️⃣ **Dimensions Library (Type-Safe Computations)**

A **custom template-based units library** (`dimensions`) ensures **compile-time unit safety** for physical values like **radians, velocity, torque, and stiffness**.

```cpp
Torque t = 1.5_Nm;                  // Newton Meter
AngularVelocity w = 10_rad / 1_s;   // Radians per Second
```

---

## Build

#### Using CMake

To build the library with CMake, follow these steps:

```sh
mkdir build && cd build
cmake ..
cmake --build .
```

The commands above will build the library along with provided examples. If you don't need the examples, run:

```sh
cmake -D BUILD_EXAMPLES=NO ..
```

---

## Setting Up CAN Communication on Linux (SocketCAN)

For Linux-based systems (Ubuntu, Raspberry Pi, Nvidia Jetson):

```sh
sudo ip link set can0 up type can bitrate 1000000
```

---

## Example Usage

A simple example using the **VelocityAccelController**:

```cpp
#include <iostream>
#include <thread>
#include <chrono>
#include <kot_motor/kot_motor.hpp>

using namespace kot_motor;

SocketCanTransport t("can0");
Motor m(t, 3, 0, config::default_motor);
VelocityAccelController c(m, {}, 1, 10);

int main() {
  t.open();
  std::cout << "start" << "\n";
  c.switchOn();

  std::this_thread::sleep_for(std::chrono::seconds(2));
  c.damper(4);
  c.velocity(5);
  c.stop();
  std::this_thread::sleep_for(std::chrono::seconds(2));

  std::cout << "stop" << "\n";
  c.switchOff();
  t.close();

  return 0;
}
```

---

## Contributing

We welcome contributions! Feel free to add new transport methods, controllers, or documentation.

---

## License

This library is open-source and licensed under **MIT License**.

---

Let us know if you build something cool with it! 🚀


