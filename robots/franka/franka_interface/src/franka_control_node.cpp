// /***************************************************************************
// * Adapted from franka_control_node.cpp (frank_ros package)

// *
// * @package: franka_interface
// * @metapackage: franka_ros_interface
// * @author: Saif Sidhik <sxs1412@bham.ac.uk>
// *

// **************************************************************************/

// /***************************************************************************
// * Copyright (c) 2019-2020, Saif Sidhik.
// * Copyright (c) 2017 Franka Emika GmbH
// *
// * Licensed under the Apache License, Version 2.0 (the "License");
// * you may not use this file except in compliance with the License.
// * You may obtain a copy of the License at
// *
// *     http://www.apache.org/licenses/LICENSE-2.0
// *
// * Unless required by applicable law or agreed to in writing, software
// * distributed under the License is distributed on an "AS IS" BASIS,
// * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// * See the License for the specific language governing permissions and
// * limitations under the License.
// **************************************************************************/

#include <algorithm>
#include <atomic>

#include <actionlib/server/simple_action_server.h>
#include <controller_manager/controller_manager.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka_hw/franka_hw.h>
#include <franka_hw/services.h>
#include <franka_msgs/ErrorRecoveryAction.h>
#include <ros/ros.h>

#include <franka_interface/motion_controller_interface.h>

using franka_hw::ServiceContainer;

int main(int argc, char** argv) {
  ros::init(argc, argv, "franka_control_node");

  ros::NodeHandle public_node_handle;
  ros::NodeHandle config_node_handle("/robot_config");
  ros::NodeHandle base_node_handle("~");

  franka_hw::FrankaHW franka_control;
  if (!franka_control.init(public_node_handle, config_node_handle)) {
    ROS_ERROR("franka_control_node: Failed to initialize FrankaHW class. Shutting down!");
    return 1;
  }

  franka::Robot& robot = franka_control.robot();

  std::atomic_bool has_error(false);

  ServiceContainer services;
    services
      .advertiseService<franka_msgs::SetJointImpedance>(base_node_handle, "/franka_ros_interface/franka_control/set_joint_impedance",
                                                        [&robot](auto&& req, auto&& res) {
                                                          return franka_hw::setJointImpedance(
                                                              robot, req, res);
                                                        })
      .advertiseService<franka_msgs::SetCartesianImpedance>(
          base_node_handle, "/franka_ros_interface/franka_control/set_cartesian_impedance",
          [&robot](auto&& req, auto&& res) {
            return franka_hw::setCartesianImpedance(robot, req, res);
          })
      .advertiseService<franka_msgs::SetEEFrame>(
          base_node_handle, "/franka_ros_interface/franka_control/set_EE_frame",
          [&robot](auto&& req, auto&& res) { return franka_hw::setEEFrame(robot, req, res); })
      .advertiseService<franka_msgs::SetKFrame>(
          base_node_handle, "/franka_ros_interface/franka_control/set_K_frame",
          [&robot](auto&& req, auto&& res) { return franka_hw::setKFrame(robot, req, res); })
      .advertiseService<franka_msgs::SetForceTorqueCollisionBehavior>(
          base_node_handle, "/franka_ros_interface/franka_control/set_force_torque_collision_behavior",
          [&robot](auto&& req, auto&& res) {
            return franka_hw::setForceTorqueCollisionBehavior(robot, req, res);
          })
      .advertiseService<franka_msgs::SetFullCollisionBehavior>(
          base_node_handle, "/franka_ros_interface/franka_control/set_full_collision_behavior",
          [&robot](auto&& req, auto&& res) {
            return franka_hw::setFullCollisionBehavior(robot, req, res);
          })
      .advertiseService<franka_msgs::SetLoad>(
          base_node_handle, "/franka_ros_interface/franka_control/set_load",
          [&robot](auto&& req, auto&& res) { return franka_hw::setLoad(robot, req, res); });

  actionlib::SimpleActionServer<franka_msgs::ErrorRecoveryAction> recovery_action_server(
      base_node_handle, "/franka_ros_interface/franka_control/error_recovery",
      [&](const franka_msgs::ErrorRecoveryGoalConstPtr&) {
        try {
          robot.automaticErrorRecovery();
          has_error = false;
          recovery_action_server.setSucceeded();
          ROS_INFO("Recovered from error");
        } catch (const franka::Exception& ex) {
          recovery_action_server.setAborted(franka_msgs::ErrorRecoveryResult(), ex.what());
        }
      },
      false);
    franka_control.update(robot.readOnce());

  boost::shared_ptr<controller_manager::ControllerManager> control_manager;

  control_manager.reset(new controller_manager::ControllerManager(&franka_control, public_node_handle)); 

  franka_interface::MotionControllerInterface motion_controller_interface_;
  motion_controller_interface_.init(public_node_handle, control_manager);

  recovery_action_server.start();

  // // Start background threads for message handling
  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok()) {
    ros::Time last_time = ros::Time::now();

    // Wait until controller has been activated or error has been recovered
    while (!franka_control.controllerActive() || has_error) {
      franka_control.update(robot.readOnce());

      ros::Time now = ros::Time::now();
      control_manager->update(now, now - last_time);
      franka_control.checkJointLimits();
      last_time = now;

      if (!ros::ok()) {
        return 0;
      }
    }

    try {
      // Run control loop. Will exit if the controller is switched.
      franka_control.control([&](const ros::Time& now, const ros::Duration& period) {
        if (period.toSec() == 0.0) {
          // Reset controllers before starting a motion
          control_manager->update(now, period, true);
          franka_control.checkJointLimits();
          franka_control.reset();
        } else {
          control_manager->update(now, period);
          franka_control.checkJointLimits();
          franka_control.enforceLimits(period);
        }
        return ros::ok();
      });
    } catch (const franka::ControlException& e) {
      ROS_ERROR("%s", e.what());
      has_error = true;
    }
  }

  return 0;

}