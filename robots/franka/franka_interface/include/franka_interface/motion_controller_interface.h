/***************************************************************************
* Adapted from sawyer_simulator package

*
* @package: franka_interface
* @metapackage: franka_ros_interface
* @author: Saif Sidhik <sxs1412@bham.ac.uk>
*

**************************************************************************/

/***************************************************************************
* Copyright (c) 2019-2020, Saif Sidhik.
* Copyright (c) 2013-2018, Rethink Robotics Inc.
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
#ifndef _FRANKA_INTERFACE____MOTION_CONTROLLER_INTERFACE_H_
#define _FRANKA_INTERFACE____MOTION_CONTROLLER_INTERFACE_H_

#include <mutex>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_box.h>
#include <franka_core_msgs/JointCommand.h>


namespace franka_interface {
  class MotionControllerInterface
  {
  public:
    /**
   * Initializes the controller manager.
   *
   * @param[in] root_node_handle Node handle in the controller_manager namespace.
   * @param[in] controller_manager the controller manager instance.
   */
    void init(ros::NodeHandle& nh,
         boost::shared_ptr<controller_manager::ControllerManager> controller_manager);

  private:
    // mutex for re-entrant calls to modeCommandCallback
    std::mutex mtx_;
    int current_mode_;

    realtime_tools::RealtimeBox< std::shared_ptr<const ros::Duration > > box_timeout_length_;
    realtime_tools::RealtimeBox< std::shared_ptr<const ros::Time > > box_cmd_timeout_;

    ros::Timer cmd_timeout_timer_;

    ros::Subscriber joint_command_timeout_sub_;
    ros::Subscriber joint_command_sub_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    std::string position_controller_name_;
    std::string velocity_controller_name_;
    std::string torque_controller_name_;
    std::string impedance_controller_name_;
    std::string trajectory_controller_name_;

    std::string default_controller_name_;
    std::string current_controller_name_;
    std::vector<std::string> all_controllers_;

    std::map<std::string,int> controller_name_to_mode_map_;
  protected:
  /**
   * Callback function to set time out to switch back to position mode. When using torque
   * mode controllers, consecutive torque commands should be sent in smaller intervals than
   * this timeout value
   *
   * @param[in] msg Float64 instance containing the desired timeout.
   */
    void jointCommandTimeoutCallback(const std_msgs::Float64 msg);

  /**
   * Callback function to choose the appropriate controller. This function checks the type
   * of controller requested by the client through the type field in the message, and 
   * starts the controller.
   *
   * @param[in] msg JointCommandConstPtr instance containing the joint commands.
   */
    void jointCommandCallback(const franka_core_msgs::JointCommandConstPtr& msg);

  /**
   * Switch controller depending on the mode specified.
   *
   * @param[in] control_mode integer value representing the type of controller to use
   */
    bool switchControllers(int control_mode);

  /**
   * Switch controller to the initially defined default controller.
   */
    bool switchToDefaultController();

   /**
   * Check if the command timeout has been violated.
   *
   * @param[in] control_mode integer value representing the type of controller to use
   */
    void commandTimeoutCheck(const ros::TimerEvent& e);


  };
}
#endif // #ifndef _FRANKA_INTERFACE____MOTION_CONTROLLER_INTERFACE_H_
