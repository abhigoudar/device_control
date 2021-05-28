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

#include "device_control_msgs/Control.h"
#include "device_control_msgs/DeviceInfo.h"

#include "ros/ros.h"

#include "device_control/ControlledLifecycleNode.hpp"

namespace device_control
{

ControlledLifecycleNode::ControlledLifecycleNode(const std::string & system_id)
: nh_(), system_id_(system_id), state_(UNCONFIGURED)
{
  device_control_sub_ = nh_.subscribe("device_control", 1, &ControlledLifecycleNode::control_callback, this);
  device_control_pub_ = nh_.advertise<device_control_msgs::Control>("device_control", 100);
  device_info_pub_ = nh_.advertise<device_control_msgs::DeviceInfo>("device_environment", 100, true);
}

bool
ControlledLifecycleNode::trigger_transition(Transition transition)
{
  switch (transition) {
    case CONFIGURE:
      if (state_ == UNCONFIGURED) {
        if (on_configure()) {
          state_ = INACTIVE;
        }
      } else {
        ROS_ERROR("CONFIGURE transition from state %d", state_);
        return false;
      }
      break;
    case ACTIVATE:
      if (state_ == INACTIVE) {
        if (on_activate()) {
          state_ = ACTIVE;
        }
      } else {
        ROS_ERROR("ACTIVATE transition from state %d", state_);
        return false;
      }
      break;
    case DEACTIVATE:
      if (state_ == ACTIVE) {
        if (on_deactivate()) {
          state_ = INACTIVE;
        }
      } else {
        ROS_ERROR("DEACTIVATE transition from state %d", state_);
        return false;
      }
      break;
  }

  return true;
}

bool
ControlledLifecycleNode::on_configure()
{
  device_control_msgs::DeviceInfo msg;
  msg.device_source = ros::this_node::getName();
  msg.topics.assign(topics_.begin(), topics_.end());

  device_info_pub_.publish(msg);

  return true;
}

bool
ControlledLifecycleNode::on_activate()
{
  return true;
}

bool
ControlledLifecycleNode::on_deactivate()
{
  return true;
}


void
ControlledLifecycleNode::control_callback(const device_control_msgs::Control::ConstPtr & msg)
{
  if (!msg->device_systems.empty() &&
    std::find(msg->device_systems.begin(), msg->device_systems.end(), ros::this_node::getName()) ==
    msg->device_systems.end())
  {
    return;
  }

  switch (msg->control_type) {
    case device_control_msgs::Control::START:
      if (state_ == INACTIVE) {
        trigger_transition(ACTIVATE);
        device_control_msgs::Control msg_reply;
        msg_reply.control_type = device_control_msgs::Control::ACK_START;
        msg_reply.stamp = ros::Time::now();
        msg_reply.device_source = ros::this_node::getName();
        device_control_pub_.publish(msg_reply);

        control_start(msg);
      } else {
        ROS_WARN_STREAM(
          "Activation requested in state " << state_);
      }
      break;

    case device_control_msgs::Control::STOP:
      if (state_ == ACTIVE) {
        trigger_transition(DEACTIVATE);
        device_control_msgs::Control msg_reply;
        msg_reply.control_type = device_control_msgs::Control::ACK_STOP;
        msg_reply.stamp = ros::Time::now();
        msg_reply.device_source =ros::this_node::getName();
        device_control_pub_.publish(msg_reply);

        control_stop(msg);
      } else {
        ROS_WARN_STREAM(
          "Deactivation requested in state " << state_);
      }
      break;

    default:
      break;
  }
}

}  // namespace device_control
