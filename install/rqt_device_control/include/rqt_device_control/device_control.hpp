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

#ifndef RQT_DEVICE_CONTROL__DEVICE_CONTROL__HPP_
#define RQT_DEVICE_CONTROL__DEVICE_CONTROL__HPP_

#include <rqt_gui_cpp/plugin.h>

#include <ui_device_control.h>

#include <map>

#include <QAction>
#include <QImage>
#include <QList>
#include <QString>
#include <QSet>
#include <QSize>
#include <QWidget>

#include <vector>

#include "device_control_msgs/msg/device_info.hpp"

#include "device_control/ControllerNode.hpp"
#include "rqt_device_control/SystemController.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rqt_device_control {


class DeviceControl
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  DeviceControl();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual void shutdownPlugin();

  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

protected slots:

  // virtual void updateTopicList();

protected:

  // virtual QSet<QString> getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports);

  // virtual void selectTopic(const QString& topic);

protected slots:

  void start_capture();
  void select_output_dir();
  void select_record_all(bool checked);
  void select_active_all(bool checked);
  void enable_ros1(int state);
  void spin_loop();
  // virtual void onTopicChanged(int index);
// 
  // virtual void onZoom1(bool checked);
// 
  // virtual void onDynamicRange(bool checked);
// 
  // virtual void saveImage();
// 
  // virtual void updateNumGridlines();
// 
  // virtual void onMousePublish(bool checked);
// 
  // virtual void onMouseLeft(int x, int y);
// 
  // virtual void onPubTopicChanged();
// 
  // virtual void onHideToolbarChanged(bool hide);
// 
  // virtual void onRotateLeft();
  // virtual void onRotateRight();

protected:

  // virtual void callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
// 
  // virtual void invertPixels(int x, int y);
// 
  // QList<int> getGridIndices(int size) const;
// 
  // virtual void overlayGrid();

  Ui::DeviceControlWidget ui_;

  QWidget* widget_;

  // image_transport::Subscriber subscriber_;

private:
  rclcpp::Subscription<device_control_msgs::msg::DeviceInfo>::SharedPtr device_env_sub_;
  std::map<std::string, SystemController*> device_env_;

  std::shared_ptr<device_control::ControllerNode> controller_node_;
  QTimer *controller_spin_timer_;
  bool capturing_ {false};

  void update_tree(const device_control_msgs::msg::DeviceInfo::SharedPtr msg);
  void control_callback(const device_control_msgs::msg::Control::SharedPtr msg);
  
  void start_roscore_bridges();
  pid_t pid_roscore_ {0}, pid_bridge1_{0}, pid_bridge2_{0};

  std::string current_output_dir_;
};

}  // namespace rqt_device_control

#endif  // RQT_DEVICE_CONTROL__Device_CONTRO__HPP_