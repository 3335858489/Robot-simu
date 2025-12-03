cmake_minimum_required(VERSION 3.22)
project(dummy_moveit_config)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

# 将 moveit_servo 设为可选
find_package(moveit_servo QUIET)

add_executable(dummy_servo_keyboard_input src/servo_keyboard_input.cpp)
target_include_directories(dummy_servo_keyboard_input PUBLIC include)
ament_target_dependencies(dummy_servo_keyboard_input std_msgs control_msgs rclcpp)

# 只有在找到 moveit_servo 时才编译这个可执行文件
if(moveit_servo_FOUND)
  add_executable(dummy_servo_cpp_interface_demo src/servo_cpp_interface_demo.cpp)
  target_include_directories(dummy_servo_cpp_interface_demo PUBLIC include)
  ament_target_dependencies(dummy_servo_cpp_interface_demo moveit_servo rclcpp)
  
  install(
    TARGETS
      dummy_servo_keyboard_input
      dummy_servo_cpp_interface_demo
    DESTINATION
      lib/${PROJECT_NAME}
  )
  message(STATUS "moveit_servo found - building servo demo")
else()
  install(
    TARGETS
      dummy_servo_keyboard_input
    DESTINATION
      lib/${PROJECT_NAME}
  )
  message(WARNING "moveit_servo not found - skipping servo demo")
endif()

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
