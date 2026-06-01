#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <limits>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace omni_robot_hardware {

class ReactiveExploreV2Node : public rclcpp::Node {
 public:
  ReactiveExploreV2Node() : Node("bt_reactive_explorer_v2_node") {
    // Basic parameters
    declare_parameter("cmd_vel_topic", "cmd_vel_safe");
    declare_parameter("front_topic", "ultrasonic/front");
    declare_parameter("back_topic", "ultrasonic/back");
    declare_parameter("right_topic", "ultrasonic/right");
    declare_parameter("left_topic", "ultrasonic/left");
    declare_parameter("imu_topic", "imu/data");
    declare_parameter("tick_rate_hz", 20.0);
    declare_parameter("forward_speed_mps", 0.045);
    declare_parameter("front_block_m", 0.20);
    declare_parameter("front_blocked_persist_s", 1.5);
    declare_parameter("back_block_m", 0.15);
    declare_parameter("wall_detect_m", 0.15);
    declare_parameter("wall_follow_distance_m", 0.10);
    declare_parameter("wall_follow_kp", 1.2);
    declare_parameter("reverse_speed_mps", 0.06);
    declare_parameter("turn_speed_rads", 0.3);
    declare_parameter("backup_time_s", 1.4);
    declare_parameter("turn_time_s", 1.8);
    declare_parameter("forward_test_time_s", 1.2);
    declare_parameter("spin_time_s", 3.2);
    declare_parameter("wait_time_s", 0.8);
    declare_parameter("tilt_stop_deg", 85.0);
    declare_parameter("impact_accel_mps2", 12.5);
    declare_parameter("impact_cooldown_s", 1.0);
    declare_parameter("bt_xml_file", "");

    std::string cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
    std::string front_topic = get_parameter("front_topic").as_string();
    std::string back_topic = get_parameter("back_topic").as_string();
    std::string right_topic = get_parameter("right_topic").as_string();
    std::string left_topic = get_parameter("left_topic").as_string();
    std::string imu_topic = get_parameter("imu_topic").as_string();
    
    tick_rate_hz_ = get_parameter("tick_rate_hz").as_double();
    forward_speed_mps_ = get_parameter("forward_speed_mps").as_double();
    front_block_m_ = get_parameter("front_block_m").as_double();
    front_blocked_persist_s_ = get_parameter("front_blocked_persist_s").as_double();
    back_block_m_ = get_parameter("back_block_m").as_double();
    wall_detect_m_ = get_parameter("wall_detect_m").as_double();
    wall_follow_distance_m_ = get_parameter("wall_follow_distance_m").as_double();
    wall_follow_kp_ = get_parameter("wall_follow_kp").as_double();
    tilt_stop_deg_ = get_parameter("tilt_stop_deg").as_double();
    impact_accel_mps2_ = get_parameter("impact_accel_mps2").as_double();
    impact_cooldown_s_ = get_parameter("impact_cooldown_s").as_double();
    
    double reverse_speed = get_parameter("reverse_speed_mps").as_double();
    double turn_speed = get_parameter("turn_speed_rads").as_double();
    double backup_time = get_parameter("backup_time_s").as_double();
    double turn_time = get_parameter("turn_time_s").as_double();
    double forward_test_time = get_parameter("forward_test_time_s").as_double();
    double spin_time = get_parameter("spin_time_s").as_double();
    double wait_time = get_parameter("wait_time_s").as_double();

    std::string bt_xml_file = get_parameter("bt_xml_file").as_string();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

    // Front sensor subscription
    front_sub_ = create_subscription<sensor_msgs::msg::Range>(
        front_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Range::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(sensor_mutex_);
          front_range_ = msg->range;
        });

    // Back sensor subscription
    back_sub_ = create_subscription<sensor_msgs::msg::Range>(
        back_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Range::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(sensor_mutex_);
          back_range_ = msg->range;
        });

    // Left sensor subscription
    left_sub_ = create_subscription<sensor_msgs::msg::Range>(
        left_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Range::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(sensor_mutex_);
          left_range_ = msg->range;
        });

    // Right sensor subscription
    right_sub_ = create_subscription<sensor_msgs::msg::Range>(
        right_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Range::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(sensor_mutex_);
          right_range_ = msg->range;
        });

    // IMU subscription
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(),
        std::bind(&ReactiveExploreV2Node::imu_callback, this, std::placeholders::_1));

    // BT Blackboard setup
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("node", this);
    blackboard_->set("forward_speed_mps", forward_speed_mps_);
    blackboard_->set("reverse_speed_mps", -std::abs(reverse_speed));
    blackboard_->set("turn_speed_rads", turn_speed);
    blackboard_->set("backup_time_s", backup_time);
    blackboard_->set("turn_time_s", turn_time);
    blackboard_->set("forward_test_time_s", forward_test_time);
    blackboard_->set("spin_time_s", spin_time);
    blackboard_->set("wait_time_s", wait_time);

    register_bt_nodes();

    if (bt_xml_file.empty()) {
      RCLCPP_ERROR(get_logger(), "bt_xml_file is empty. Cannot start.");
      return;
    }

    try {
      tree_ = std::make_unique<BT::Tree>(
          factory_.createTreeFromFile(bt_xml_file, blackboard_));
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(get_logger(), "Failed to load BT XML: %s", ex.what());
      return;
    }

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / tick_rate_hz_),
        std::bind(&ReactiveExploreV2Node::on_timer, this));

    RCLCPP_INFO(get_logger(), "V2 Reactive Explorer started.");
  }

  // --- Core API for BT Nodes ---
  void publish_cmd(double vx, double wz) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx;
    cmd.angular.z = wz;
    cmd_pub_->publish(cmd);
  }

  double get_forward_speed() const { return forward_speed_mps_; }
  
  double get_front_range() const {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    return front_range_;
  }

  double get_back_range() const {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    return back_range_;
  }

  double get_side_ranges(double& left, double& right) const {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    left = left_range_;
    right = right_range_;
    return std::min(left, right);
  }

  void get_imu_state(bool& tilt, bool& impact) const {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    tilt = tilt_unsafe_;
    impact = impact_detected_;
  }

  void clear_impact() {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    impact_detected_ = false;
  }

  double get_front_block_m() const { return front_block_m_; }
  double get_front_blocked_persist_s() const { return front_blocked_persist_s_; }
  double get_back_block_m() const { return back_block_m_; }
  double get_wall_detect_m() const { return wall_detect_m_; }
  double get_wall_follow_distance_m() const { return wall_follow_distance_m_; }
  double get_wall_follow_kp() const { return wall_follow_kp_; }

  void record_front_block() {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (first_front_block_time_.seconds() == 0.0) {
      first_front_block_time_ = now();
    }
  }

  void clear_front_block_time() {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    first_front_block_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  double get_front_block_duration_s() const {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (first_front_block_time_.seconds() == 0.0) return 0.0;
    return (now() - first_front_block_time_).seconds();
  }

 private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;

    double pitch_rad = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    double roll_rad = std::atan2(ay, az);

    double pitch_deg = pitch_rad * 180.0 / M_PI;
    double roll_deg = roll_rad * 180.0 / M_PI;

    double magnitude = std::sqrt(ax * ax + ay * ay + az * az);

    std::lock_guard<std::mutex> lock(sensor_mutex_);
    tilt_unsafe_ = (std::abs(pitch_deg) > tilt_stop_deg_ || std::abs(roll_deg) > tilt_stop_deg_);

    if (magnitude > impact_accel_mps2_) {
      if ((now() - last_impact_time_).seconds() > impact_cooldown_s_) {
        impact_detected_ = true;
        last_impact_time_ = now();
        RCLCPP_WARN(get_logger(), "IMPACT DETECTED! acc=%.2f", magnitude);
      }
    }
  }
  // --- PHASE 1: Real Actions ---
  struct DriveForwardAction : public BT::ActionNodeBase {
    DriveForwardAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override {
      auto* node = config().blackboard->get<ReactiveExploreV2Node*>("node");
      node->publish_cmd(node->get_forward_speed(), 0.0);
      return BT::NodeStatus::RUNNING;
    }
    void halt() override {
      auto* node = config().blackboard->get<ReactiveExploreV2Node*>("node");
      node->publish_cmd(0.0, 0.0);
    }
  };

  struct StopAction : public BT::ActionNodeBase {
    StopAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override {
      auto* node = config().blackboard->get<ReactiveExploreV2Node*>("node");
      node->publish_cmd(0.0, 0.0);
      return BT::NodeStatus::SUCCESS; // Must return SUCCESS so it doesn't freeze the tree!
    }
    void halt() override {
      auto* node = config().blackboard->get<ReactiveExploreV2Node*>("node");
      node->publish_cmd(0.0, 0.0);
    }
  };

  // --- PHASE 2: Real Obstacle Detection & Reversing ---
  struct FrontBlockedCondition : public BT::ConditionNode {
    FrontBlockedCondition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override {
      auto* node = config().blackboard->get<ReactiveExploreV2Node*>("node");
      double r = node->get_front_range();
      if (std::isfinite(r) && r <= node->get_front_block_m()) {
        node->record_front_block();
        RCLCPP_INFO_THROTTLE(node->get_logger(), *(node->get_clock()), 1000, 
                             "FrontBlocked! range=%.3f <= %.3f", r, node->get_front_block_m());
        return BT::NodeStatus::SUCCESS;
      }
      node->clear_front_block_time();
      return BT::NodeStatus::FAILURE;
    }
  };

  struct TimedTwistAction : public BT::StatefulActionNode {
    TimedTwistAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config) {}
    
    static BT::PortsList providedPorts() {
      return { BT::InputPort<double>("vx"), BT::InputPort<double>("wz"), BT::InputPort<double>("duration_s") };
    }

    BT::NodeStatus onStart() override {
      auto* node = config().blackboard->get<ReactiveExploreV2Node*>("node");
      start_time_ = node->now();
      RCLCPP_INFO(node->get_logger(), "TimedTwist started");
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
      auto* node = config().blackboard->get<ReactiveExploreV2Node*>("node");
      double vx = 0.0;
      double wz = 0.0;
      double duration_s = 0.0;

      if (!getInput("vx", vx) || !getInput("wz", wz) || !getInput("duration_s", duration_s)) {
        RCLCPP_ERROR(node->get_logger(), "TimedTwist: getInput failed! Check blackboard mapping.");
        return BT::NodeStatus::FAILURE;
      }

      // Safety check: stop reversing if we get too close to something behind us
      if (vx < 0.0) {
        double r = node->get_back_range();
        if (std::isfinite(r) && r <= node->get_back_block_m()) {
          RCLCPP_WARN_THROTTLE(node->get_logger(), *(node->get_clock()), 1000,
                               "TimedTwist aborted: back blocked! range=%.3f <= %.3f", r, node->get_back_block_m());
          node->publish_cmd(0.0, 0.0);
          return BT::NodeStatus::SUCCESS; // Stop early, safety triggered
        }
      }

      double elapsed = (node->now() - start_time_).seconds();
      if (elapsed >= duration_s) {
        RCLCPP_INFO(node->get_logger(), "TimedTwist completed (%.2fs)", elapsed);
        node->publish_cmd(0.0, 0.0);
        return BT::NodeStatus::SUCCESS;
      }

      node->publish_cmd(vx, wz);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
      auto* node = config().blackboard->get<ReactiveExploreV2Node*>("node");
      node->publish_cmd(0.0, 0.0);
    }

   private:
    rclcpp::Time start_time_;
  };

  // --- PHASE 3: Turn In Place ---
  struct TurnInPlaceAction : public BT::StatefulActionNode {
    TurnInPlaceAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config) {}
    
    static BT::PortsList providedPorts() { 
      return { BT::InputPort<double>("duration_s") }; 
    }

    BT::NodeStatus onStart() override {
      auto* node = config().blackboard->get<ReactiveExploreV2Node*>("node");
      start_time_ = node->now();
      
      // Determine turn direction: simply alternate for now
      static bool turn_left = true;
      dir_ = turn_left ? 1.0 : -1.0;
      turn_left = !turn_left;
      
      RCLCPP_INFO(node->get_logger(), "TurnInPlace started (dir=%.1f)", dir_);
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
      auto* node = config().blackboard->get<ReactiveExploreV2Node*>("node");
      double duration_s = 0.0;
      if (!getInput("duration_s", duration_s)) {
        RCLCPP_ERROR(node->get_logger(), "TurnInPlace: getInput failed!");
        return BT::NodeStatus::FAILURE;
      }

      double elapsed1 = (node->now() - start_time_).seconds();
      // Don't allow early exit for the first 1.5s — commit to turning
      double min_turn_s = 1.5;

      // INTERRUPT ROTATION: If the front is clear, stop turning early!
      double r = node->get_front_range();
      double clear_threshold = node->get_front_block_m() * 1.5; // 50% margin
      if (elapsed1 > min_turn_s && (!std::isfinite(r) || r > clear_threshold)) {
        RCLCPP_INFO(node->get_logger(), "TurnInPlace interrupted: front is clear! (range=%.3f)", r);
        node->clear_front_block_time();
        node->publish_cmd(0.0, 0.0);
        return BT::NodeStatus::SUCCESS;
      }

      double elapsed = (node->now() - start_time_).seconds();
      if (elapsed >= duration_s) {
        RCLCPP_INFO(node->get_logger(), "TurnInPlace completed (%.2fs)", elapsed);
        node->clear_front_block_time();
        node->publish_cmd(0.0, 0.0);
        return BT::NodeStatus::SUCCESS;
      }

      double wz = dir_ * config().blackboard->get<double>("turn_speed_rads");
      node->publish_cmd(0.0, wz);
      return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
      auto* node = config().blackboard->get<ReactiveExploreV2Node*>("node");
      node->publish_cmd(0.0, 0.0);
    }

   private:
    rclcpp::Time start_time_;
    double dir_{1.0};
  };

  // --- PHASE 4: Wall Following & Stuck Recovery ---
  struct FrontBlockedLongCondition : public BT::ConditionNode {
    FrontBlockedLongCondition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override {
      auto* node = config().blackboard->get<ReactiveExploreV2Node*>("node");
      if (node->get_front_block_duration_s() > node->get_front_blocked_persist_s()) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *(node->get_clock()), 2000, "FrontBlockedLong triggered!");
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::FAILURE;
    }
  };

  struct WallDetectedCondition : public BT::ConditionNode {
    WallDetectedCondition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override {
      auto* node = config().blackboard->get<ReactiveExploreV2Node*>("node");
      double left, right;
      double min_side = node->get_side_ranges(left, right);
      if (std::isfinite(min_side) && min_side <= node->get_wall_detect_m()) {
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::FAILURE;
    }
  };

  struct ShouldRandomEscapeCondition : public BT::ConditionNode {
    ShouldRandomEscapeCondition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override {
      // 10% chance to randomly escape a wall instead of following it
      bool escape = (static_cast<double>(rand()) / RAND_MAX) < 0.10;
      return escape ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  };

  struct WallFollowAction : public BT::ActionNodeBase {
    WallFollowAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    
    BT::NodeStatus tick() override {
      auto* node = config().blackboard->get<ReactiveExploreV2Node*>("node");
      double left, right;
      node->get_side_ranges(left, right);
      
      double err = 0.0;
      double target = node->get_wall_follow_distance_m();
      
      if (std::isfinite(left) && (!std::isfinite(right) || left < right)) {
        err = target - left; // Following left wall
      } else if (std::isfinite(right)) {
        err = right - target; // Following right wall
      } else {
        return BT::NodeStatus::FAILURE; // No wall to follow
      }

      double kp = node->get_wall_follow_kp();
      double wz = kp * err;
      
      // Cap rotation speed
      double max_turn = config().blackboard->get<double>("turn_speed_rads");
      wz = std::clamp(wz, -max_turn, max_turn);
      
      node->publish_cmd(node->get_forward_speed(), wz);
      return BT::NodeStatus::RUNNING;
    }
    
    void halt() override {
      auto* node = config().blackboard->get<ReactiveExploreV2Node*>("node");
      node->publish_cmd(0.0, 0.0);
    }
  };

  // --- PHASE 5: Safety Monitors ---
  struct TiltUnsafeCondition : public BT::ConditionNode {
    TiltUnsafeCondition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override {
      auto* node = config().blackboard->get<ReactiveExploreV2Node*>("node");
      bool tilt, impact;
      node->get_imu_state(tilt, impact);
      if (tilt) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *(node->get_clock()), 1000, "Tilt Unsafe!");
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::FAILURE;
    }
  };

  struct ImpactDetectedCondition : public BT::ConditionNode {
    ImpactDetectedCondition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override {
      auto* node = config().blackboard->get<ReactiveExploreV2Node*>("node");
      bool tilt, impact;
      node->get_imu_state(tilt, impact);
      if (impact) {
        node->clear_impact();
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::FAILURE;
    }
  };

  void register_bt_nodes() {
    // Phase 1
    factory_.registerNodeType<DriveForwardAction>("DriveForward");
    factory_.registerNodeType<StopAction>("Stop");

    // Phase 2
    factory_.registerNodeType<FrontBlockedCondition>("FrontBlocked");
    factory_.registerNodeType<TimedTwistAction>("TimedTwist");

    // Phase 3
    factory_.registerNodeType<TurnInPlaceAction>("TurnInPlace");

    // Phase 4
    factory_.registerNodeType<FrontBlockedLongCondition>("FrontBlockedLong");
    factory_.registerNodeType<WallDetectedCondition>("WallDetected");
    factory_.registerNodeType<ShouldRandomEscapeCondition>("ShouldRandomEscape");
    factory_.registerNodeType<WallFollowAction>("WallFollow");

    // Phase 5
    factory_.registerNodeType<TiltUnsafeCondition>("TiltUnsafe");
    factory_.registerNodeType<ImpactDetectedCondition>("ImpactDetected");
  }

  void on_timer() {
    if (tree_) {
      tree_->tickRoot();
    }
  }

  double tick_rate_hz_{20.0};
  double forward_speed_mps_{0.045};
  double front_block_m_{0.20};
  double front_blocked_persist_s_{1.5};
  double back_block_m_{0.15};
  double wall_detect_m_{0.15};
  double wall_follow_distance_m_{0.10};
  double wall_follow_kp_{1.2};
  double tilt_stop_deg_{85.0};
  double impact_accel_mps2_{12.5};
  double impact_cooldown_s_{1.0};
  
  mutable std::mutex sensor_mutex_;
  double front_range_{std::numeric_limits<double>::infinity()};
  double back_range_{std::numeric_limits<double>::infinity()};
  double left_range_{std::numeric_limits<double>::infinity()};
  double right_range_{std::numeric_limits<double>::infinity()};
  bool tilt_unsafe_{false};
  bool impact_detected_{false};
  rclcpp::Time last_impact_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time first_front_block_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr back_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<BT::Tree> tree_;
};

}  // namespace omni_robot_hardware

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<omni_robot_hardware::ReactiveExploreV2Node>());
  rclcpp::shutdown();
  return 0;
}
