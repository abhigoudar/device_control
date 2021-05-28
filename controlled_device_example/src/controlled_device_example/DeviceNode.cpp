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


#include <string>

#include "controlled_device_example/DeviceNode.hpp"

#include "sensor_msgs/Imu.h"
#include "mocap_msgs/ImusInfo.h"

#include "ros/ros.h"

namespace controlled_device_example
{

DeviceNode::DeviceNode()
: ControlledLifecycleNode(ros::this_node::getName())
{
  imu_pub_ = create_publisher<sensor_msgs::Imu>("mopcap_imu_data", 1000);
  loop_timer_ = nh_.createTimer(ros::Rate(30), &DeviceNode::callback_imu_data, this, false, false);
}

bool
DeviceNode::on_configure()
{
  // Initialize you system (open device, send mode...)

  return ControlledLifecycleNode::on_configure();
}

bool
DeviceNode::on_activate()
{
  // Start the capture. In this case, we will set a timer to produce
  // the fake data
  loop_timer_.start();

  return ControlledLifecycleNode::on_activate();
}

bool
DeviceNode::on_deactivate()
{
  // Stop the capture
  loop_timer_.stop();

  return ControlledLifecycleNode::on_deactivate();
}

void
DeviceNode::callback_imu_data(const ros::TimerEvent&)
{
  if (imu_pub_.getNumSubscribers() > 0) {
    sensor_msgs::Imu imu_msg;
    // Fill the data
    imu_pub_.publish(imu_msg);
  }
}

}  // namespace controlled_device_example

