#include <iostream>
#include <thread>
#include <chrono>
#include <kot_motor/kot_motor.hpp>

using namespace kot_motor;

SocketCanTransport t("can0");
Motor m(t, 3, 0, config::default_motor);
VelocityAccelController c(m, {}, 1, 10);

int main() {
  std::this_thread::sleep_for(std::chrono::seconds(2));

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




