#include "omni_robot_hardware/pigpio_client.hpp"

#include <pigpiod_if2.h>

namespace omni_robot_hardware {

PigpioClient::PigpioClient() {
  pi_ = pigpio_start(nullptr, nullptr);
}

PigpioClient::~PigpioClient() {
  if (pi_ >= 0) {
    pigpio_stop(pi_);
  }
}

}  // namespace omni_robot_hardware
