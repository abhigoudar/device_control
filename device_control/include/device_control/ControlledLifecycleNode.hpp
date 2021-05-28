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

#ifndef DEVICE_CONTROL__CONTROLLEDLIFECYCLENODE_HPP_
#define DEVICE_CONTROL__CONTROLLEDLIFECYCLENODE_HPP_

#include <string>
#include <set>
#include <memory>

#include "device_control_msgs/Control.h"
#include "device_control_msgs/DeviceInfo.h"
#include "ros/ros.h"

namespace device_control
{

enum State {UNCONFIGURED, INACTIVE, ACTIVE};
enum Transition {CONFIGURE, ACTIVATE, DEACTIVATE};

class ControlledLifecycleNode
{
public:
  explicit ControlledLifecycleNode(const std::string & system_id);

  bool trigger_transition(Transition transition);

protected:
  ros::NodeHandle nh_;

  template<typename MessageT>
  ros::Publisher
  create_publisher(const std::string & topic_name, int queue, bool latched = false)
  {
    if (topic_name != "device_control" && topic_name != "device_environment") {
      topics_.insert(topic_name);
    }

    return nh_.advertise<MessageT>(topic_name, queue, latched);
  }

  State state_;
  std::string system_id_;

  virtual bool on_configure();
  virtual bool on_activate();
  virtual bool on_deactivate();

  virtual void control_start(const device_control_msgs::Control::ConstPtr & msg) {(void)msg;}
  virtual void control_stop(const device_control_msgs::Control::ConstPtr & msg) {(void)msg;}

  std::set<std::string> topics_;

private:
  ros::Subscriber device_control_sub_;
  ros::Publisher device_control_pub_;
  ros::Publisher device_info_pub_;

  void control_callback(const device_control_msgs::Control::ConstPtr & msg);
};

}  // namespace device_control

#endif  // DEVICE_CONTROL__CONTROLLEDLIFECYCLENODE_HPP_
