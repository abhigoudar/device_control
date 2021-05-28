// Copyright 2020 Intelligent Robotics Lab
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
#include <vector>

#include "device_control_msgs/msg/control.hpp"
#include "rclcpp/rclcpp.hpp"

#include "device_control/ControllerNode.hpp"

namespace device_control
{

using std::placeholders::_1;
using namespace std::chrono_literals;

ControllerNode::ControllerNode()
: Node("device_controller")
{
  control_sub_ = create_subscription<device_control_msgs::msg::Control>(
    "device_control", rclcpp::QoS(100).reliable(),
    std::bind(&ControllerNode::control_callback, this, _1));

  control_pub_ = create_publisher<device_control_msgs::msg::Control>(
    "device_control", rclcpp::QoS(100).reliable());
}

void
ControllerNode::control_callback(const device_control_msgs::msg::Control::SharedPtr msg)
{
  switch (msg->control_type) {
    case device_control_msgs::msg::Control::ACK_START:
      {
        auto elapsed = now() - msg->stamp;

        if (elapsed < 1ms) {
          RCLCPP_INFO(
            get_logger(),
            "[%s] start elapsed = %lf secs", msg->device_source.c_str(), elapsed.seconds());
        } else if (elapsed < 200ms) {
          RCLCPP_WARN(
            get_logger(),
            "[%s] start elapsed = %lf secs", msg->device_source.c_str(), elapsed.seconds());
        } else {
          RCLCPP_ERROR(
            get_logger(),
            "[%s] start elapsed = %lf secs", msg->device_source.c_str(), elapsed.seconds());
        }
      }
      break;

    case device_control_msgs::msg::Control::ACK_STOP:
      {
        auto elapsed = now() - msg->stamp;

        if (elapsed < 1ms) {
          RCLCPP_INFO(
            get_logger(),
            "[%s] stop elapsed = %lf secs", msg->device_source.c_str(), elapsed.seconds());
        } else if (elapsed < 200ms) {
          RCLCPP_WARN(
            get_logger(),
            "[%s] stop = %lf secs", msg->device_source.c_str(), elapsed.seconds());
        } else {
          RCLCPP_ERROR(
            get_logger(),
            "[%s] stop = %lf secs", msg->device_source.c_str(), elapsed.seconds());
        }
      }
      break;

    default:
      break;
  }
}

void
ControllerNode::start_system(
  const std::string & session_id,
  const std::vector<std::string> & device_systems)
{
  device_control_msgs::msg::Control msg;
  msg.control_type = device_control_msgs::msg::Control::START;
  msg.stamp = now();
  msg.device_source = get_name();
  msg.session_id = session_id;
  msg.device_systems = device_systems;
  control_pub_->publish(msg);
}

void
ControllerNode::stop_system()
{
  device_control_msgs::msg::Control msg;
  msg.control_type = device_control_msgs::msg::Control::STOP;
  msg.stamp = now();
  msg.device_source = get_name();

  control_pub_->publish(msg);
}

}  // namespace device_control
