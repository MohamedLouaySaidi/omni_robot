#include "omni_robot_hardware/hc_sr04.hpp"
#include "omni_robot_hardware/pigpio_client.hpp"

#include <algorithm>
#include <pigpiod_if2.h>
#include <limits>
#include <memory>
#include <string>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <deque>

namespace omni_robot_hardware {

struct SensorConfig {
  std::string name;
  std::string frame_id;
  unsigned trig{0};
  unsigned echo{0};
  bool enabled{false};
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub;
  std::deque<float> history;
};

class UltrasonicNode : public rclcpp::Node {
 public:
  UltrasonicNode() : Node("ultrasonic_node") {
    declare_parameter("min_range_m", 0.02);
    declare_parameter("max_range_m", 4.0);
    declare_parameter("field_of_view", 0.26);
    declare_parameter("publish_rate_hz", 3.0);
    declare_parameter("inter_sensor_delay_ms", 80);
    declare_parameter("treat_too_close_as_min_range", true);
    declare_parameter("log_debug", true);
    declare_parameter("filter_window", 5);
    declare_parameter("trim_size", 1);

    min_range_m_ = get_parameter("min_range_m").as_double();
    max_range_m_ = get_parameter("max_range_m").as_double();
    fov_ = get_parameter("field_of_view").as_double();
    treat_close_as_min_ = get_parameter("treat_too_close_as_min_range").as_bool();
    const double rate = get_parameter("publish_rate_hz").as_double();
    inter_sensor_delay_ms_ = std::max(
      0, static_cast<int>(get_parameter("inter_sensor_delay_ms").as_int()));
    log_debug_ = get_parameter("log_debug").as_bool();
    filter_window_ = get_parameter("filter_window").as_int();
    trim_size_ = get_parameter("trim_size").as_int();

    pigpio_ = std::make_unique<PigpioClient>();
    if (!pigpio_->ok()) {
      RCLCPP_FATAL(get_logger(),
                   "pigpio_start failed — run: sudo systemctl start pigpiod");
      throw std::runtime_error("pigpio_start failed");
    }

    load_sensors();

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        std::bind(&UltrasonicNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "Ultrasonic node at %.1f Hz (pigpiod)", rate);
  }

 private:
  void load_sensors() {
    const std::vector<std::pair<std::string, std::pair<unsigned, unsigned>>> defs = {
        {"front", {17, 27}}, {"back", {22, 10}},
        {"left", {9, 11}},   {"right", {25, 8}},
    };

    const int pi = pigpio_->handle();

    for (const auto& [name, pins] : defs) {
      const std::string en_param = "enable_" + name;
      declare_parameter(en_param, name == "front" || name == "back");

      SensorConfig cfg;
      cfg.name = name;
      cfg.frame_id = name + "_ultrasonic_link";
      cfg.trig = pins.first;
      cfg.echo = pins.second;
      cfg.enabled = get_parameter(en_param).as_bool();

      if (!cfg.enabled) {
        continue;
      }

      set_mode(pi, cfg.trig, PI_OUTPUT);
      set_mode(pi, cfg.echo, PI_INPUT);
      set_pull_up_down(pi, cfg.echo, PI_PUD_OFF);
      gpio_write(pi, cfg.trig, 0);

      cfg.pub = create_publisher<sensor_msgs::msg::Range>(
          "ultrasonic/" + name, rclcpp::SensorDataQoS());

      sensors_.push_back(cfg);
      RCLCPP_INFO(get_logger(), "Enabled %s trig=%u echo=%u", name.c_str(),
                  cfg.trig, cfg.echo);
    }
  }

  void on_timer() {
    const int pi = pigpio_->handle();

    for (auto& s : sensors_) {
      const auto reading = measure_hc_sr04(
          pi, s.trig, s.echo, static_cast<float>(min_range_m_),
          static_cast<float>(max_range_m_));

      sensor_msgs::msg::Range msg;
      msg.header.stamp = now();
      msg.header.frame_id = s.frame_id;
      msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
      msg.field_of_view = static_cast<float>(fov_);
      msg.min_range = static_cast<float>(min_range_m_);
      msg.max_range = static_cast<float>(max_range_m_);

      float raw_range = std::numeric_limits<float>::infinity();
      switch (reading.status) {
        case HcSr04Status::kOk:
          raw_range = reading.distance_m;
          break;
        case HcSr04Status::kTooClose:
          raw_range = treat_close_as_min_
                          ? static_cast<float>(min_range_m_)
                          : std::numeric_limits<float>::quiet_NaN();
          break;
        default:
          raw_range = std::numeric_limits<float>::infinity();
          break;
      }

      if (std::isfinite(raw_range) && raw_range > 0.0f) {
        s.history.push_back(raw_range);
      } else {
        s.history.clear();
      }
      while (static_cast<int>(s.history.size()) > filter_window_) {
        s.history.pop_front();
      }
      if (s.history.empty()) {
        msg.range = std::numeric_limits<float>::infinity();
      } else {
        std::vector<float> sorted(s.history.begin(), s.history.end());
        std::sort(sorted.begin(), sorted.end());
        
        int trim = std::clamp(trim_size_, 0, std::max(0, static_cast<int>(sorted.size() - 1) / 2));
        float sum = 0.0f;
        int count = 0;
        for (int i = trim; i < static_cast<int>(sorted.size()) - trim; ++i) {
          sum += sorted[i];
          count++;
        }
        msg.range = (count > 0) ? (sum / count) : sorted[sorted.size() / 2];
      }

      s.pub->publish(msg);
      if (log_debug_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "%s: status=%d range=%.3f",
                             s.name.c_str(),
                             static_cast<int>(reading.status),
                             msg.range);
      }
      if (inter_sensor_delay_ms_ > 0) {
        usleep(static_cast<unsigned>(inter_sensor_delay_ms_) * 1000U);
      }
    }
  }

  std::unique_ptr<PigpioClient> pigpio_;
  double min_range_m_{0.02};
  double max_range_m_{4.0};
  double fov_{0.26};
  bool treat_close_as_min_{true};
  int inter_sensor_delay_ms_{80};
  bool log_debug_{true};
  int filter_window_{5};
  int trim_size_{1};
  std::vector<SensorConfig> sensors_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace omni_robot_hardware

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<omni_robot_hardware::UltrasonicNode>());
  rclcpp::shutdown();
  return 0;
}
