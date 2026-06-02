/**
 * ultrasonic_node.cpp  —  multithreaded version
 * =============================================
 * One std::thread per enabled sensor.
 * Each thread fires its HC-SR04, filters, and publishes independently.
 *
 * Crosstalk mitigation: threads are staggered by (index × stagger_ms)
 * before entering their loop, so sensors never fire at the same instant.
 *
 * Parameters (unchanged from original):
 *   publish_rate_hz          – measurement + publish rate per sensor  (default 10)
 *   stagger_ms               – offset between thread start times      (default 60)
 *   inter_sensor_delay_ms    – kept for compatibility, ignored here
 *   filter_window            – trimmed-mean window size               (default 5)
 *   trim_size                – samples trimmed from each end          (default 1)
 *   min_range_m / max_range_m / field_of_view
 *   treat_too_close_as_min_range
 *   enable_front / enable_back / enable_left / enable_right
 *   log_debug
 */

#include "omni_robot_hardware/hc_sr04.hpp"
#include "omni_robot_hardware/pigpio_client.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <pigpiod_if2.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

namespace omni_robot_hardware {

// ── per-sensor state ──────────────────────────────────────────────────────
struct SensorState {
  std::string name;
  std::string frame_id;
  unsigned trig{0};
  unsigned echo{0};
  bool enabled{false};

  // publisher (created on the ROS node thread, read-only after that)
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub;

  // filter history  (only touched inside the sensor's own thread)
  std::deque<float> history;
};

// ─────────────────────────────────────────────────────────────────────────
class UltrasonicNode : public rclcpp::Node {
 public:
  UltrasonicNode() : Node("ultrasonic_node") {
    // ── declare parameters ───────────────────────────────────────────────
    declare_parameter("min_range_m",                   0.02);
    declare_parameter("max_range_m",                   4.0);
    declare_parameter("field_of_view",                 0.26);
    declare_parameter("publish_rate_hz",               10.0);
    declare_parameter("stagger_ms",                    60);   // NEW
    declare_parameter("inter_sensor_delay_ms",         80);   // kept, unused
    declare_parameter("treat_too_close_as_min_range",  true);
    declare_parameter("log_debug",                     true);
    declare_parameter("filter_window",                 5);
    declare_parameter("trim_size",                     1);

    min_range_m_      = get_parameter("min_range_m").as_double();
    max_range_m_      = get_parameter("max_range_m").as_double();
    fov_              = get_parameter("field_of_view").as_double();
    publish_rate_hz_  = get_parameter("publish_rate_hz").as_double();
    stagger_ms_       = get_parameter("stagger_ms").as_int();
    treat_close_as_min_ = get_parameter("treat_too_close_as_min_range").as_bool();
    log_debug_        = get_parameter("log_debug").as_bool();
    filter_window_    = get_parameter("filter_window").as_int();
    trim_size_        = get_parameter("trim_size").as_int();

    // ── pigpio ───────────────────────────────────────────────────────────
    pigpio_ = std::make_unique<PigpioClient>();
    if (!pigpio_->ok()) {
      RCLCPP_FATAL(get_logger(),
                   "pigpio_start failed — run: sudo systemctl start pigpiod");
      throw std::runtime_error("pigpio_start failed");
    }

    // ── build sensor list ────────────────────────────────────────────────
    load_sensors();

    // ── spawn one thread per enabled sensor ──────────────────────────────
    running_.store(true);
    int index = 0;
    for (auto& s : sensors_) {
      threads_.emplace_back(&UltrasonicNode::sensor_thread, this,
                            std::ref(s), index);
      ++index;
    }

    RCLCPP_INFO(get_logger(),
                "Ultrasonic node: %zu sensor thread(s) at %.1f Hz, stagger %d ms",
                sensors_.size(), publish_rate_hz_, stagger_ms_);
  }

  ~UltrasonicNode() override {
    running_.store(false);
    for (auto& t : threads_) {
      if (t.joinable()) t.join();
    }
  }

 private:
  // ── sensor setup ─────────────────────────────────────────────────────
  void load_sensors() {
    const std::vector<std::pair<std::string, std::pair<unsigned, unsigned>>> defs = {
        {"front", {17, 27}},
        {"back",  {22, 10}},
        {"left",  {9,  11}},
        {"right", {25, 8}},
    };

    const int pi = pigpio_->handle();

    for (const auto& [name, pins] : defs) {
      const std::string en_param = "enable_" + name;
      declare_parameter(en_param, name == "front" || name == "back");

      SensorState s;
      s.name     = name;
      s.frame_id = name + "_ultrasonic_link";
      s.trig     = pins.first;
      s.echo     = pins.second;
      s.enabled  = get_parameter(en_param).as_bool();

      if (!s.enabled) continue;

      set_mode(pi, s.trig, PI_OUTPUT);
      set_mode(pi, s.echo, PI_INPUT);
      set_pull_up_down(pi, s.echo, PI_PUD_OFF);
      gpio_write(pi, s.trig, 0);

      s.pub = create_publisher<sensor_msgs::msg::Range>(
          "ultrasonic/" + name, rclcpp::SensorDataQoS());

      sensors_.push_back(std::move(s));
      RCLCPP_INFO(get_logger(), "Enabled %s trig=%u echo=%u",
                  name.c_str(), pins.first, pins.second);
    }
  }

  // ── per-sensor thread ────────────────────────────────────────────────
  void sensor_thread(SensorState& s, int index) {
    // stagger: thread N sleeps N × stagger_ms before starting its loop
    // this prevents multiple sensors from firing at the same moment
    if (index > 0) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(index * stagger_ms_));
    }

    const int pi = pigpio_->handle();
    const std::chrono::duration<double> period(1.0 / publish_rate_hz_);

    RCLCPP_INFO(get_logger(),
                "Thread [%s] started (stagger=%d ms)",
                s.name.c_str(), index * stagger_ms_);

    while (running_.load()) {
      const auto tick_start = std::chrono::steady_clock::now();

      // ── measure ────────────────────────────────────────────────────
      const auto reading = measure_hc_sr04(
          pi, s.trig, s.echo,
          static_cast<float>(min_range_m_),
          static_cast<float>(max_range_m_));

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

      // ── filter (trimmed mean) ──────────────────────────────────────
      if (std::isfinite(raw_range) && raw_range > 0.0f) {
        s.history.push_back(raw_range);
      } else {
        s.history.clear();
      }
      while (static_cast<int>(s.history.size()) > filter_window_) {
        s.history.pop_front();
      }

      float filtered = std::numeric_limits<float>::infinity();
      if (!s.history.empty()) {
        std::vector<float> sorted(s.history.begin(), s.history.end());
        std::sort(sorted.begin(), sorted.end());
        const int trim = std::clamp(
            trim_size_,
            0,
            std::max(0, static_cast<int>(sorted.size() - 1) / 2));
        float sum = 0.0f;
        int   cnt = 0;
        for (int i = trim; i < static_cast<int>(sorted.size()) - trim; ++i) {
          sum += sorted[i];
          ++cnt;
        }
        filtered = (cnt > 0) ? (sum / cnt) : sorted[sorted.size() / 2];
      }

      // ── publish ────────────────────────────────────────────────────
      sensor_msgs::msg::Range msg;
      msg.header.stamp    = now();   // rclcpp::Node::now() is thread-safe
      msg.header.frame_id = s.frame_id;
      msg.radiation_type  = sensor_msgs::msg::Range::ULTRASOUND;
      msg.field_of_view   = static_cast<float>(fov_);
      msg.min_range       = static_cast<float>(min_range_m_);
      msg.max_range       = static_cast<float>(max_range_m_);
      msg.range           = filtered;
      s.pub->publish(msg);   // rclcpp publishers are thread-safe

      if (log_debug_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "[%s] raw=%.3f filtered=%.3f",
                             s.name.c_str(), raw_range, filtered);
      }

      // ── sleep for remainder of period ──────────────────────────────
      const auto elapsed = std::chrono::steady_clock::now() - tick_start;
      const auto sleep_for = period - elapsed;
      if (sleep_for > std::chrono::duration<double>(0.0)) {
        std::this_thread::sleep_for(sleep_for);
      }
    }

    RCLCPP_INFO(get_logger(), "Thread [%s] stopped", s.name.c_str());
  }

  // ── members ──────────────────────────────────────────────────────────
  std::unique_ptr<PigpioClient> pigpio_;

  double   min_range_m_{0.02};
  double   max_range_m_{4.0};
  double   fov_{0.26};
  double   publish_rate_hz_{10.0};
  int      stagger_ms_{60};
  bool     treat_close_as_min_{true};
  bool     log_debug_{true};
  int      filter_window_{5};
  int      trim_size_{1};

  std::vector<SensorState>  sensors_;
  std::vector<std::thread>  threads_;
  std::atomic<bool>         running_{false};
};

}  // namespace omni_robot_hardware

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<omni_robot_hardware::UltrasonicNode>());
  rclcpp::shutdown();
  return 0;
}