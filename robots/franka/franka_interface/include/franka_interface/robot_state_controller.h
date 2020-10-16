/***************************************************************************
* Adapted from robot_state_controller.cpp (franka_ros package)

*
* @package: franka_interface
* @metapackage: franka_ros_interface
* @author: Saif Sidhik <sxs1412@bham.ac.uk>
*

**************************************************************************/

/***************************************************************************
* Copyright (c) 2019-2020, Saif Sidhik
* Copyright (c) 2017 Franka Emika GmbH
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**************************************************************************/


#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>
#include <franka_core_msgs/RobotState.h>
#include <franka_core_msgs/EndPointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>
#include <tf2_msgs/TFMessage.h>
#include <controller_manager/controller_manager.h>
#include <Eigen/Dense>

namespace franka_interface {

/**
 * Custom controller to publish the robot state as ROS topics.
 */
class CustomFrankaStateController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaStateInterface,
                                                            franka_hw::FrankaModelInterface> {
 public:
  /**
   * Creates an instance of a CustomFrankaStateController.
   */
  CustomFrankaStateController() = default;

  /**
   * Initializes the controller with interfaces and publishers.
   *
   * @param[in] robot_hardware RobotHW instance to get a franka_hw::FrankaStateInterface from.
   * @param[in] root_node_handle Node handle in the controller_manager namespace.
   * @param[in] controller_node_handle Node handle in the controller namespace.
   */
  bool init(hardware_interface::RobotHW* robot_hardware,
            ros::NodeHandle& root_node_handle,
            ros::NodeHandle& controller_node_handle) override;

  /**
   * Reads the current robot state from the franka_hw::FrankaStateInterface and publishes it.
   *
   * @param[in] time Current ROS time.
   * @param[in] period Time since the last update.
   */
  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  void publishFrankaState(const ros::Time& time);
  void publishJointStates(const ros::Time& time);
  void publishTransforms(const ros::Time& time);
  void publishEndPointState(const ros::Time& time);

  std::string arm_id_;

  franka_hw::FrankaStateInterface* franka_state_interface_{};
  std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_{};
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

  realtime_tools::RealtimePublisher<tf2_msgs::TFMessage> publisher_transforms_;
  realtime_tools::RealtimePublisher<franka_core_msgs::RobotState> publisher_franka_state_;
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> publisher_joint_states_;
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> publisher_joint_states_desired_;
  realtime_tools::RealtimePublisher<franka_core_msgs::EndPointState> publisher_tip_state_;
  franka_hw::TriggerRate trigger_publish_;
  franka::RobotState robot_state_;
  uint64_t sequence_number_ = 0;
  std::vector<std::string> joint_names_;
};

}  // namespace franka_interface
