// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONTROLLED_DEVICE_EXAMPLE__DEVICENODE_HPP__
#define CONTROLLED_DEVICE_EXAMPLE__DEVICENODE_HPP__


#include "sensor_msgs/Imu.h"

#include "device_control/ControlledLifecycleNode.hpp"

#include "ros/ros.h"

namespace controlled_device_example
{

class DeviceNode : public device_control::ControlledLifecycleNode
{
public:
  explicit DeviceNode();

  bool on_configure();
  bool on_activate();
  bool on_deactivate();

  void device_cleanup();
private:
  void callback_imu_data(const ros::TimerEvent&);

  ros::Publisher imu_pub_;
  ros::Timer loop_timer_;
};

}  // namespace controlled_device_example


#endif  // MOCAP4ROS2_TECHNAID__MCSNODE_HPP__
