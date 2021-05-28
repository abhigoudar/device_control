cmake_minimum_required(VERSION 3.5)
project(device_control)

set(dependencies 
  roscpp
  std_msgs
  device_control_msgs
)

find_package(catkin REQUIRED COMPONENTS ${dependencies})
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${dependencies}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/device_control/ControlledLifecycleNode.cpp
  # src/device_control/ControllerNode.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
