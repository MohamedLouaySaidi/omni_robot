cmake_minimum_required(VERSION 3.16)
project(omni_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS 2 & system dependencies ──────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# pigpiod C library (GPIO + PWM via pigpiod daemon)
find_library(PIGPIO_LIB pigpio REQUIRED)
find_library(PIGPIOD_IF2_LIB pigpiod_if2 REQUIRED)

# ── Shared libraries ──────────────────────────────────────────────────────
add_library(hc_sr04 SHARED src/hc_sr04.cpp)
target_include_directories(hc_sr04 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(hc_sr04 PUBLIC ${PIGPIO_LIB} ${PIGPIOD_IF2_LIB})
target_compile_features(hc_sr04 PUBLIC cxx_std_17)

# pigpio connection helper
add_library(pigpio_client SHARED src/pigpio_client.cpp)
target_include_directories(pigpio_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pigpio_client PUBLIC ${PIGPIOD_IF2_LIB})
target_compile_features(pigpio_client PUBLIC cxx_std_17)

# L298N tank drive abstraction
add_library(tank_drive SHARED src/tank_drive.cpp)
target_include_directories(tank_drive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(tank_drive PUBLIC ${PIGPIOD_IF2_LIB})
target_compile_features(tank_drive PUBLIC cxx_std_17)

# ── Executable nodes ────────────────────────────────────────────

# HC-SR04 sensor publisher (4 sensors, multithreaded)
add_executable(ultrasonic_node src/ultrasonic_node.cpp)
target_link_libraries(ultrasonic_node hc_sr04 pigpio_client)
ament_target_dependencies(ultrasonic_node rclcpp sensor_msgs)

# ADXL345 IMU publisher (I2C via Linux kernel driver)
add_executable(imu_node src/imu_node.cpp)
ament_target_dependencies(imu_node rclcpp sensor_msgs)

# L298N motor driver node
add_executable(tank_drive_node src/tank_drive_node.cpp)
target_link_libraries(tank_drive_node tank_drive pigpio_client)
ament_target_dependencies(tank_drive_node rclcpp geometry_msgs)

# BT Reactive Explorer V2 — main autonomous navigation node
add_executable(bt_reactive_explorer_v2_node src/bt_reactive_explorer_v2_node.cpp)
ament_target_dependencies(bt_reactive_explorer_v2_node rclcpp geometry_msgs sensor_msgs behaviortree_cpp_v3)


install(TARGETS hc_sr04 pigpio_client tank_drive ultrasonic_node imu_node
  tank_drive_node bt_reactive_explorer_v2_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

ament_package()
