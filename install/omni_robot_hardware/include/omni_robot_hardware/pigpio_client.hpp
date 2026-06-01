#pragma once

namespace omni_robot_hardware {

/** Connects to local pigpiod (start with: sudo systemctl start pigpiod). */
class PigpioClient {
 public:
  PigpioClient();
  ~PigpioClient();

  PigpioClient(const PigpioClient&) = delete;
  PigpioClient& operator=(const PigpioClient&) = delete;

  int handle() const { return pi_; }
  bool ok() const { return pi_ >= 0; }

 private:
  int pi_{-1};
};

}  // namespace omni_robot_hardware
