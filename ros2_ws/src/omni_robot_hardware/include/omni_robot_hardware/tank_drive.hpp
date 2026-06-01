#pragma once

namespace omni_robot_hardware {

/** OSOYOO Model-X tank drive: independent left/right PWM (ENA / ENB). */
class TankDrive {
 public:
  TankDrive(int pi, unsigned ena_right, unsigned enb_left, unsigned in1,
            unsigned in2, unsigned in3, unsigned in4, unsigned pwm_freq_hz = 1000);

  void stop();
  /** left/right command in [-1, 1] (sign = direction). */
  void set_arcade(float left, float right, unsigned max_pwm = 255);

 private:
  void set_side(unsigned in_a, unsigned in_b, unsigned enable, float command,
                unsigned max_pwm);

  int pi_{-1};
  unsigned ena_right_;
  unsigned enb_left_;
  unsigned in1_;
  unsigned in2_;
  unsigned in3_;
  unsigned in4_;
};

}  // namespace omni_robot_hardware
