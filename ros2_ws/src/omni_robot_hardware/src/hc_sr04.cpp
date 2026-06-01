#include "omni_robot_hardware/hc_sr04.hpp"

#include <pigpiod_if2.h>
#include <unistd.h>

namespace omni_robot_hardware {

namespace {

constexpr unsigned kTriggerPulseUs = 10;
constexpr unsigned kEchoSettleUs = 50;
constexpr unsigned kTimeoutUs = 30000;
constexpr float kUsToM = 1.0f / 58000.0f;

void delay_us(unsigned us) {
  usleep(us);
}

}  // namespace

HcSr04Reading measure_hc_sr04(int pi, unsigned trig_bcm, unsigned echo_bcm,
                              float min_m, float max_m) {
  HcSr04Reading r;

  gpio_write(pi, trig_bcm, 0);
  delay_us(2);
  gpio_write(pi, trig_bcm, 1);
  delay_us(kTriggerPulseUs);
  gpio_write(pi, trig_bcm, 0);
  delay_us(kEchoSettleUs);

  const uint32_t deadline = get_current_tick(pi) + kTimeoutUs;

  while (gpio_read(pi, echo_bcm) == 0) {
    if (static_cast<int32_t>(get_current_tick(pi) - deadline) >= 0) {
      r.status = HcSr04Status::kTimeoutWaitHigh;
      return r;
    }
  }
  const uint32_t echo_start = get_current_tick(pi);

  while (gpio_read(pi, echo_bcm) == 1) {
    if (static_cast<int32_t>(get_current_tick(pi) - deadline) >= 0) {
      r.status = HcSr04Status::kTimeoutWaitLow;
      return r;
    }
  }
  const uint32_t echo_end = get_current_tick(pi);

  r.pulse_us = echo_end - echo_start;
  r.distance_m = static_cast<float>(r.pulse_us) * kUsToM;

  if (r.distance_m < min_m) {
    r.status = HcSr04Status::kTooClose;
  } else if (r.distance_m > max_m) {
    r.status = HcSr04Status::kOutOfRange;
  } else {
    r.status = HcSr04Status::kOk;
  }
  return r;
}

}  // namespace omni_robot_hardware
