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

#include "device_control_msgs/msg/control.hpp"
#include "device_control_msgs/msg/device_info.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "device_control/ControlledLifecycleNode.hpp"

namespace device_control
{

using std::placeholders::_1;

ControlledLifecycleNode::ControlledLifecycleNode(const std::string & system_id)
: LifecycleNode(system_id)
{
  device_control_sub_ = create_subscription<device_control_msgs::msg::Control>(
    "device_control", rclcpp::QoS(100).reliable(),
    std::bind(&ControlledLifecycleNode::control_callback, this, _1));

  device_control_pub_ = create_publisher<device_control_msgs::msg::Control>(
    "device_control", rclcpp::QoS(100).reliable());

  device_info_pub_ = create_publisher<device_control_msgs::msg::DeviceInfo>(
    "device_environment", rclcpp::QoS(1000).reliable().transient_local().keep_all());

  device_control_pub_->on_activate();
  device_info_pub_->on_activate();
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
ControlledLifecycleNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  device_control_msgs::msg::DeviceInfo msg;
  msg.device_source = get_name();
  msg.topics.assign(topics_.begin(), topics_.end());

  device_info_pub_->publish(msg);

  return CallbackReturnT::SUCCESS;
}
CallbackReturnT
ControlledLifecycleNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControlledLifecycleNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControlledLifecycleNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControlledLifecycleNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}


void
ControlledLifecycleNode::control_callback(const device_control_msgs::msg::Control::SharedPtr msg)
{
  if (!msg->device_systems.empty() &&
    std::find(msg->device_systems.begin(), msg->device_systems.end(), get_name()) ==
    msg->device_systems.end())
  {
    return;
  }

  switch (msg->control_type) {
    case device_control_msgs::msg::Control::START:
      if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        device_control_msgs::msg::Control msg_reply;
        msg_reply.control_type = device_control_msgs::msg::Control::ACK_START;
        msg_reply.stamp = now();
        msg_reply.device_source = get_name();
        device_control_pub_->publish(msg_reply);

        control_start(msg);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Activation requested in state: %s ", get_current_state().label());
      }
      break;

    case device_control_msgs::msg::Control::STOP:
      if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        device_control_msgs::msg::Control msg_reply;
        msg_reply.control_type = device_control_msgs::msg::Control::ACK_STOP;
        msg_reply.stamp = now();
        msg_reply.device_source = get_name();
        device_control_pub_->publish(msg_reply);

        control_stop(msg);
      }
      break;

    default:
      break;
  }
}

}  // namespace device_control
