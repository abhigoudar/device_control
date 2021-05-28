project(controlled_device_example)

cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(device_control REQUIRED)
find_package(device_control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    device_control
    device_control_msgs
    sensor_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/controlled_device_example/DeviceNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME})
add_executable(device_main
  src/device_main.cpp
)
ament_target_dependencies(device_main ${dependencies})
target_link_libraries(device_main ${PROJECT_NAME})



install(TARGETS
  ${PROJECT_NAME}
  device_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
