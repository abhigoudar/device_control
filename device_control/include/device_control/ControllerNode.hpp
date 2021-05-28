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

#ifndef DEVICE_CONTROL__CONTROLLERNODE_HPP_
#define DEVICE_CONTROL__CONTROLLERNODE_HPP_

#include <vector>
#include <string>

#include "device_control_msgs/msg/control.hpp"
#include "rclcpp/rclcpp.hpp"

namespace device_control
{

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode(
    std::function<void(
      const device_control_msgs::msg::Control::SharedPtr msg)> callback =
    [](const device_control_msgs::msg::Control::SharedPtr) {});  // NOLINT(runtime/explicit)

  void start_system(
    const std::string & session_id,
    const std::vector<std::string> & device_systems);
  void stop_system();

private:
  void control_callback(const device_control_msgs::msg::Control::SharedPtr msg);

  std::function<void(const device_control_msgs::msg::Control::SharedPtr msg)> callback_;

  rclcpp::Subscription<device_control_msgs::msg::Control>::SharedPtr control_sub_;
  rclcpp::Publisher<device_control_msgs::msg::Control>::SharedPtr control_pub_;
};

}  // namespace device_control

#endif  // DEVICE_CONTROL__CONTROLLERNODE_HPP_
