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
#include <set>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "device_control/AuxiliarNode.hpp"

namespace device_control
{

using std::placeholders::_1;

AuxiliarNode::AuxiliarNode(const std::string & system_id)
: ControlledLifecycleNode(system_id)
{
  declare_parameter("topics", std::vector<std::string>{});
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
AuxiliarNode::on_configure(const rclcpp_lifecycle::State & state)
{
  auto topics_params = get_parameter("topics").as_string_array();
  topics_ = std::set<std::string>(topics_params.begin(), topics_params.end());

  return ControlledLifecycleNode::on_configure(state);
}

void
AuxiliarNode::control_start(const device_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
  RCLCPP_INFO(get_logger(), "System [%s] started with topics:", get_name());
  for (const auto & topic : topics_) {
    RCLCPP_INFO(get_logger(), "  - [%s]",  topic);
  }
}

void
AuxiliarNode::control_stop(const device_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
  RCLCPP_INFO(get_logger(), "System [%s] stopped", get_name());
}

}  // namespace device_control
