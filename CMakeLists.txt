cmake_minimum_required(VERSION 3.8)
project(my_hardware_pwm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === ROS2 dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# === WiringPi Library finden ===
find_library(WIRINGPI_LIB wiringPi)
if(NOT WIRINGPI_LIB)
  message(FATAL_ERROR "wiringPi library not found on your system.")
endif()

# === Executables bauen ===
add_executable(my_bosch_bldc src/my_bosch_bldc.cpp)
add_executable(pump_controller src/pump_controller.cpp)

# Includes
target_include_directories(my_bosch_bldc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(pump_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Compiler Features
target_compile_features(my_bosch_bldc PUBLIC c_std_99 cxx_std_17)
target_compile_features(pump_controller PUBLIC cxx_std_17)

# ROS2 Dependencies
ament_target_dependencies(my_bosch_bldc rclcpp std_msgs)
ament_target_dependencies(pump_controller rclcpp std_msgs)

# WiringPi linken
target_link_libraries(my_bosch_bldc ${WIRINGPI_LIB})
target_link_libraries(pump_controller ${WIRINGPI_LIB})

# Installation
install(TARGETS my_bosch_bldc pump_controller
  DESTINATION lib/${PROJECT_NAME})

# Tests (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()