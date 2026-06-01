#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace omni_robot_hardware {

namespace {

constexpr uint8_t kRegDevId = 0x00;
constexpr uint8_t kRegPowerCtl = 0x2D;
constexpr uint8_t kRegDataFormat = 0x31;
constexpr uint8_t kRegDatX0 = 0x32;
constexpr uint8_t kDevIdExpected = 0xE5;
constexpr float kGPerLsb = 0.004f;
constexpr float kGravity = 9.80665f;

}  // namespace

class ImuNode : public rclcpp::Node {
 public:
  ImuNode() : Node("imu_node") {
    declare_parameter("i2c_device", "/dev/i2c-1");
    declare_parameter("i2c_address", 0x53);
    declare_parameter("publish_rate_hz", 50.0);
    declare_parameter("frame_id", "imu_link");

    frame_id_ = get_parameter("frame_id").as_string();
    const auto dev = get_parameter("i2c_device").as_string();
    const int addr = get_parameter("i2c_address").as_int();

    fd_ = ::open(dev.c_str(), O_RDWR);
    if (fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "Cannot open %s", dev.c_str());
      throw std::runtime_error("i2c open failed");
    }
    if (::ioctl(fd_, I2C_SLAVE, addr) < 0) {
      RCLCPP_FATAL(get_logger(), "I2C_SLAVE failed for 0x%02X", addr);
      throw std::runtime_error("i2c slave failed");
    }
    if (!init_adxl345()) {
      RCLCPP_ERROR(get_logger(),
                   "ADXL345 init failed. IMU node will stay alive but publish no data. "
                   "Check wiring / I2C address.");
      imu_ok_ = false;
    }

    pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());

    const double rate = get_parameter("publish_rate_hz").as_double();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        std::bind(&ImuNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "ADXL345 IMU node at %.0f Hz (accel only)", rate);
  }

  ~ImuNode() override {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

 private:
  bool i2c_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return ::write(fd_, buf, 2) == 2;
  }

  bool i2c_read_reg(uint8_t reg, uint8_t* out, size_t len) {
    if (::write(fd_, &reg, 1) != 1) {
      return false;
    }
    return static_cast<size_t>(::read(fd_, out, len)) == len;
  }

  bool init_adxl345() {
    uint8_t id = 0;
    if (!i2c_read_reg(kRegDevId, &id, 1) || id != kDevIdExpected) {
      RCLCPP_ERROR(get_logger(), "Bad DEVID 0x%02X", id);
      return false;
    }
    return i2c_write_reg(kRegDataFormat, 0x0B) &&
           i2c_write_reg(kRegPowerCtl, 0x08);
  }

  bool read_accel_g(float& x, float& y, float& z) {
    uint8_t raw[6] = {};
    if (!i2c_read_reg(kRegDatX0, raw, 6)) {
      return false;
    }
    const int16_t xi = static_cast<int16_t>(raw[0] | (raw[1] << 8));
    const int16_t yi = static_cast<int16_t>(raw[2] | (raw[3] << 8));
    const int16_t zi = static_cast<int16_t>(raw[4] | (raw[5] << 8));
    x = static_cast<float>(xi) * kGPerLsb;
    y = static_cast<float>(yi) * kGPerLsb;
    z = static_cast<float>(zi) * kGPerLsb;
    return true;
  }

  void on_timer() {
    if (!imu_ok_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "IMU disabled (init failed)");
      return;
    }
    float x_g = 0, y_g = 0, z_g = 0;
    if (!read_accel_g(x_g, y_g, z_g)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ADXL345 read failed");
      return;
    }

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    msg.linear_acceleration.x = x_g * kGravity;
    msg.linear_acceleration.y = y_g * kGravity;
    msg.linear_acceleration.z = z_g * kGravity;

    // No gyro — orientation unknown
    msg.orientation_covariance[0] = -1.0;
    msg.angular_velocity_covariance[0] = -1.0;

    pub_->publish(msg);
  }

  int fd_{-1};
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool imu_ok_{true};
};

}  // namespace omni_robot_hardware

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<omni_robot_hardware::ImuNode>());
  rclcpp::shutdown();
  return 0;
}
