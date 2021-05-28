cmake_minimum_required(VERSION 3.5)
project(device_control_msgs)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
   DIRECTORY msg
   FILES 
      Control.msg
      DeviceInfo.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)