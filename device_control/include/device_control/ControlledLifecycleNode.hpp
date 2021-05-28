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

#include "device_control_msgs/msg/control.hpp"
#include "device_control_msgs/msg/device_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace device_control
{
class ControlledLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ControlledLifecycleNode(const std::string & system_id);

protected:
  template<typename MessageT, typename AllocatorT = std::allocator<void>>
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>>
  create_publisher(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options = (
      rclcpp_lifecycle::create_default_publisher_options<AllocatorT>()
    )
  )
  {
    if (topic_name != "device_control" && topic_name != "device_environment") {
      topics_.insert(topic_name);
    }

    using PublisherT = rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>;
    return rclcpp::create_publisher<MessageT, AllocatorT, PublisherT>(
      *this,
      topic_name,
      qos,
      options);
  }

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  virtual void control_start(const device_control_msgs::msg::Control::SharedPtr msg) {(void)msg;}
  virtual void control_stop(const device_control_msgs::msg::Control::SharedPtr msg) {(void)msg;}

  std::set<std::string> topics_;

private:
  void control_callback(const device_control_msgs::msg::Control::SharedPtr msg);

  rclcpp::Subscription<device_control_msgs::msg::Control>::SharedPtr device_control_sub_;
  rclcpp_lifecycle::LifecyclePublisher<device_control_msgs::msg::Control>::SharedPtr
    device_control_pub_;
  rclcpp_lifecycle::LifecyclePublisher<device_control_msgs::msg::DeviceInfo>::SharedPtr
    device_info_pub_;
};

}  // namespace device_control

#endif  // DEVICE_CONTROL__CONTROLLEDLIFECYCLENODE_HPP_
