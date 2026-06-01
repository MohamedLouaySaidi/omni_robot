#include "omni_robot_hardware/tank_drive.hpp"

#include <pigpiod_if2.h>

#include <algorithm>
#include <cmath>

namespace omni_robot_hardware {

TankDrive::TankDrive(int pi, unsigned ena_right, unsigned enb_left, unsigned in1,
                     unsigned in2, unsigned in3, unsigned in4, unsigned pwm_freq_hz)
    : pi_(pi),
      ena_right_(ena_right),
      enb_left_(enb_left),
      in1_(in1),
      in2_(in2),
      in3_(in3),
      in4_(in4) {
  set_mode(pi_, in1_, PI_OUTPUT);
  set_mode(pi_, in2_, PI_OUTPUT);
  set_mode(pi_, in3_, PI_OUTPUT);
  set_mode(pi_, in4_, PI_OUTPUT);
  set_mode(pi_, ena_right_, PI_OUTPUT);
  set_mode(pi_, enb_left_, PI_OUTPUT);
  set_PWM_frequency(pi_, ena_right_, pwm_freq_hz);
  set_PWM_frequency(pi_, enb_left_, pwm_freq_hz);
  stop();
}

void TankDrive::set_side(unsigned in_a, unsigned in_b, unsigned enable,
                         float command, unsigned max_pwm) {
  command = std::max(-1.0f, std::min(1.0f, command));
  const unsigned duty =
      static_cast<unsigned>(std::lround(std::fabs(command) * max_pwm));

  if (command > 0.0f) {
    gpio_write(pi_, in_a, 1);
    gpio_write(pi_, in_b, 0);
  } else if (command < 0.0f) {
    gpio_write(pi_, in_a, 0);
    gpio_write(pi_, in_b, 1);
  } else {
    gpio_write(pi_, in_a, 0);
    gpio_write(pi_, in_b, 0);
  }
  set_PWM_dutycycle(pi_, enable, duty);
}

void TankDrive::set_arcade(float left, float right, unsigned max_pwm) {
  set_side(in3_, in4_, enb_left_, left, max_pwm);
  set_side(in1_, in2_, ena_right_, right, max_pwm);
}

void TankDrive::stop() {
  set_arcade(0.0f, 0.0f, 0);
}

}  // namespace omni_robot_hardware
