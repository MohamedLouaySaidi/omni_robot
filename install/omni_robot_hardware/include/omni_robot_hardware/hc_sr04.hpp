#pragma once

#include <cstdint>

namespace omni_robot_hardware {

enum class HcSr04Status {
  kOk,
  kTimeoutWaitHigh,
  kTimeoutWaitLow,
  kOutOfRange,
  kTooClose,
};

struct HcSr04Reading {
  float distance_m{0.0f};
  uint32_t pulse_us{0};
  HcSr04Status status{HcSr04Status::kTimeoutWaitHigh};
};

/** Requires pigpiod running and a valid pigpio_start() handle. */
HcSr04Reading measure_hc_sr04(int pi, unsigned trig_bcm, unsigned echo_bcm,
                              float min_m = 0.02f, float max_m = 4.0f);

}  // namespace omni_robot_hardware
