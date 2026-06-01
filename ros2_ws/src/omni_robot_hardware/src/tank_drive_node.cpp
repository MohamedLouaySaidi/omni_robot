#include "omni_robot_hardware/pigpio_client.hpp"
#include "omni_robot_hardware/tank_drive.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace omni_robot_hardware {

class TankDriveNode : public rclcpp::Node {
 public:
  TankDriveNode() : Node("tank_drive_node") {
    declare_parameter("ena_right", 12);
    declare_parameter("enb_left", 13);
    declare_parameter("in1", 5);
    declare_parameter("in2", 6);
    declare_parameter("in3", 16);
    declare_parameter("in4", 26);
    declare_parameter("pwm_frequency_hz", 1000);
    declare_parameter("max_pwm", 200);
    declare_parameter("min_pwm", 0);
    declare_parameter("max_linear_mps", 0.5);
    declare_parameter("max_angular_rads", 1.0);
    declare_parameter("cmd_timeout_s", 0.5);
    declare_parameter("control_rate_hz", 20.0);
    declare_parameter("log_debug", true);
    declare_parameter("cmd_topic", "cmd_vel_safe");

    pigpio_ = std::make_unique<PigpioClient>();
    if (!pigpio_->ok()) {
      RCLCPP_FATAL(get_logger(),
                   "pigpio_start failed — run: sudo systemctl start pigpiod");
      throw std::runtime_error("pigpio_start failed");
    }

    const unsigned pwm_freq =
        static_cast<unsigned>(get_parameter("pwm_frequency_hz").as_int());
    drive_ = std::make_unique<TankDrive>(
        pigpio_->handle(),
        static_cast<unsigned>(get_parameter("ena_right").as_int()),
        static_cast<unsigned>(get_parameter("enb_left").as_int()),
        static_cast<unsigned>(get_parameter("in1").as_int()),
        static_cast<unsigned>(get_parameter("in2").as_int()),
        static_cast<unsigned>(get_parameter("in3").as_int()),
        static_cast<unsigned>(get_parameter("in4").as_int()),
        pwm_freq);

    max_pwm_ = static_cast<unsigned>(get_parameter("max_pwm").as_int());
    min_pwm_ = static_cast<unsigned>(get_parameter("min_pwm").as_int());
    max_linear_ = static_cast<float>(get_parameter("max_linear_mps").as_double());
    max_angular_ =
        static_cast<float>(get_parameter("max_angular_rads").as_double());
    cmd_timeout_s_ = get_parameter("cmd_timeout_s").as_double();

    const auto cmd_topic = get_parameter("cmd_topic").as_string();
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_topic, 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          last_cmd_ = *msg;
          last_cmd_time_ = now();
          if (log_debug_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                                 "cmd_vel rx: vx=%.3f wz=%.3f",
                                 msg->linear.x, msg->angular.z);
          }
        });

    const double rate = get_parameter("control_rate_hz").as_double();
    last_cmd_time_ = now();
    log_debug_ = get_parameter("log_debug").as_bool();

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        std::bind(&TankDriveNode::on_control, this));

    RCLCPP_INFO(get_logger(),
                "Tank drive ready (topic=%s, max_pwm=%u, min_pwm=%u). Start pigpiod first.",
                cmd_topic.c_str(), max_pwm_, min_pwm_);
  }

  ~TankDriveNode() override { drive_->stop(); }

 private:
  void on_control() {
    const double age = (now() - last_cmd_time_).seconds();
    if (age > cmd_timeout_s_) {
      drive_->stop();
      if (log_debug_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "cmd_vel timeout (age=%.3fs) -> stop", age);
      }
      return;
    }

    float vx = static_cast<float>(last_cmd_.linear.x);
    float wz = static_cast<float>(last_cmd_.angular.z);

    vx = std::max(-max_linear_, std::min(max_linear_, vx));
    wz = std::max(-max_angular_, std::min(max_angular_, wz));

    const float v_norm = (max_linear_ > 0.0f) ? (vx / max_linear_) : 0.0f;
    const float w_norm = (max_angular_ > 0.0f) ? (wz / max_angular_) : 0.0f;
    float left = std::max(-1.0f, std::min(1.0f, v_norm - w_norm));
    float right = std::max(-1.0f, std::min(1.0f, v_norm + w_norm));

    if (max_pwm_ > 0 && min_pwm_ > 0) {
      const float min_norm = static_cast<float>(min_pwm_) /
                             static_cast<float>(max_pwm_);
      auto apply_min = [min_norm](float v) {
        if (std::fabs(v) < 1e-3f) {
          return 0.0f;
        }
        if (std::fabs(v) < min_norm) {
          return (v > 0.0f) ? min_norm : -min_norm;
        }
        return v;
      };
      left = apply_min(left);
      right = apply_min(right);
    }

    drive_->set_arcade(left, right, max_pwm_);
    if (log_debug_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                           "drive: left=%.2f right=%.2f pwm=%u",
                           left, right, max_pwm_);
    }
  }

  std::unique_ptr<PigpioClient> pigpio_;
  std::unique_ptr<TankDrive> drive_;
  unsigned max_pwm_{200};
  unsigned min_pwm_{0};
  float max_linear_{0.5f};
  float max_angular_{1.0f};
  double cmd_timeout_s_{0.5};
  bool log_debug_{true};
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace omni_robot_hardware

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<omni_robot_hardware::TankDriveNode>());
  rclcpp::shutdown();
  return 0;
}
