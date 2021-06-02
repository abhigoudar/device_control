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


#include "controlled_device_example/DeviceNode.hpp"

#include "sensor_msgs/msg/imu.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace controlled_device_example
{

using namespace std::chrono_literals;

DeviceNode::DeviceNode()
: ControlledLifecycleNode("controlled_device_example")
{
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    "mocap_imu_data", rclcpp::SensorDataQoS().reliable());
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
DeviceNode::on_configure(const rclcpp_lifecycle::State & state)
{
  // Initialize you system (open device, send mode...)

  return ControlledLifecycleNode::on_configure(state);
}

CallbackReturnT
DeviceNode::on_activate(const rclcpp_lifecycle::State & state)
{
  // Start the device. In this case, we will set a timer to produce
  // the fake data
  loop_timer_ = create_wall_timer(30ms, std::bind(&DeviceNode::callback_imu_data, this));

  imu_pub_->on_activate();
  return ControlledLifecycleNode::on_activate(state);
}

CallbackReturnT
DeviceNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  // Stop the device
  loop_timer_ = nullptr;

  return ControlledLifecycleNode::on_deactivate(state);
}

CallbackReturnT
DeviceNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  // Deactivate device
  return ControlledLifecycleNode::on_cleanup(state);
}

CallbackReturnT
DeviceNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  // Deactivate device
  return ControlledLifecycleNode::on_shutdown(state);
}

void
DeviceNode::callback_imu_data()
{
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = now();
  // Fill data
  imu_pub_->publish(imu_msg);
}

}  // namespace controlled_device_example
