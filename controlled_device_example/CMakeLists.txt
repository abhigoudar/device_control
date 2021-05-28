project(controlled_device_example)

cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)

set(dependencies 
  roscpp
  device_control
  mocap_msgs
  sensor_msgs
)

find_package(catkin REQUIRED COMPONENTS ${dependencies})
catkin_package(CATKIN_DEPENDS ${dependencies})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/controlled_device_example/DeviceNode.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(device_main
  src/device_main.cpp
)
target_link_libraries(device_main ${catkin_LIBRARIES} ${PROJECT_NAME})



install(TARGETS
  ${PROJECT_NAME}
  device_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
