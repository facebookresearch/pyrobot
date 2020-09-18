/*******************************************************************************
* Copyright (c) Facebook, Inc. and its affiliates.
* This source code is licensed under the MIT license found in the
* LICENSE file in the root directory of this source tree.

* Copyright 2018 ROBOTIS CO., LTD.
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
*******************************************************************************/

/*
 * Modified from dynamixel_workbench_controllers/dynamixel_workbench_controllers.cpp
 * Authors: Adithya Murali, Tao Chen, Dhiraj Gandhi
 */

#include <stdlib.h>
#include <unistd.h>
#include "locobot_control/locobot_controller.h"

static const uint8_t CURRENT_CONTROL_MODE                  = 0;
static const uint8_t POSITION_CONTROL_MODE                 = 3;
// based on https://github.com/ROBOTIS-GIT/dynamixel-workbench/blob/676880b4900877e4d6b68f21b87b0153cf04fbe3/dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox/dynamixel_item.cpp#L813
static const float torque_to_current_value_ratio           = 149.795386991;

LoCoBotController::LoCoBotController()
    : node_handle_(""),
    gripper_state_(-1),
    gripper_cmd_(-1),
    is_hardware_ok_(true),
    is_initialized_(false),
    num_motors_(-1),
    prev_gripper_state_(-1) {

  read_period_ = node_handle_.param<double>("dxl_read_period", 0.010f);
  write_period_ = node_handle_.param<double>("dxl_write_period", 0.010f);
  pub_period_ = node_handle_.param<double>("publish_period", 0.010f);

  bool use_camera = node_handle_.param<bool>("use_camera", false);
  bool use_vslam = node_handle_.param<bool>("use_vslam", false);
  bool use_arm = node_handle_.param<bool>("use_arm", false);
  torque_control_ = node_handle_.param<bool>("torque_control", false);

  if (use_arm) {
    use_group_["arm"] = true;
    use_group_["gripper"] = true;
  } else {
    use_group_["arm"] = false;
    use_group_["gripper"] = false;
  }

  if (use_camera || use_vslam) {
    use_group_["camera"] = true;
  } else {
    use_group_["camera"] = false;
  }

  dxl_wb_ = new DynamixelWorkbench;

}

LoCoBotController::~LoCoBotController() {

  if (use_group_["arm"]) {
    if (!torque_control_) {
      std::map<std::string, float> restJntsMap;
      float restJnts[5] = {0, -1.30, 1.617, 0.5, 0};
      restJntsMap["joint_1"] = restJnts[0];
      restJntsMap["joint_2"] = restJnts[1];
      restJntsMap["joint_3"] = restJnts[2];
      restJntsMap["joint_4"] = restJnts[3];
      restJntsMap["joint_5"] = restJnts[4];
      bool needRest = false;
      for (int i = 0; i < 5; i++) {
        float currentAngle = joint_state_msg_.position[i];
        float desiredAngle = restJntsMap[joint_state_msg_.name[i]];
        if (fabs(currentAngle - desiredAngle) > 0.3) {
          needRest = true;
          break;
        }
      }

      if (needRest) {
        printf("Going to rest joint... \n");
        // Going to this joint is just to avoid collisions in some edge cases
        sensor_msgs::JointState *intermediate_joint = new sensor_msgs::JointState();
        intermediate_joint->position = {0, 0.1135, 0.9603, -0.9235, -0.0015};
        goalJointPositionCallback(sensor_msgs::JointState::ConstPtr(intermediate_joint));

        usleep(3000000);

        sensor_msgs::JointState *rest_joint = new sensor_msgs::JointState();
        rest_joint->position.assign(restJnts, restJnts + 5);
        goalJointPositionCallback(sensor_msgs::JointState::ConstPtr(rest_joint));

        usleep(4000000);

      }
      stopDynamixels();
    }
  }
}

bool LoCoBotController::initWorkbench(const std::string port_name, const uint32_t baud_rate) {
  bool result = false;
  const char *log;

  result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
  if (!result) {
    ROS_ERROR("%s", log);
  }

  return result;
}

bool LoCoBotController::getDynamixelsInfo(const std::string yaml_file) {
  YAML::Node dynamixel;
  dynamixel = YAML::LoadFile(yaml_file.c_str());

  if (dynamixel == NULL)
    return false;

  for (YAML::const_iterator it_group = dynamixel.begin(); it_group != dynamixel.end(); it_group++) {
    std::string group_name = it_group->first.as<std::string>();
    if (!use_group_[group_name]) {
      ROS_INFO("NOT loading group %s from yaml file", group_name.c_str());
      continue;
    } else {
      ROS_INFO("Loading group %s from yaml file", group_name.c_str());
    }

    YAML::Node group_items = dynamixel[group_name];
    for (YAML::const_iterator it_file = group_items.begin(); it_file != group_items.end(); it_file++) {
      std::string name = it_file->first.as<std::string>();
      if (name.size() == 0) {
        continue;
      }
      group_motors_[group_name].push_back(name);
      YAML::Node item = group_items[name];
      for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++) {
        std::string item_name = it_item->first.as<std::string>();
        int32_t value = it_item->second.as<int32_t>();

        if (item_name == "ID") {
          dynamixel_name_2ids_[name] = value;
        }

        ItemValue item_value = {item_name, value};
        std::pair<std::string, ItemValue> info(name, item_value);

        dynamixel_info_.push_back(info);
      }
    }
  }

  int state_list_id = 0;
  for (auto const &dxl:dynamixel_name_2ids_) {
    motor_id_2state_list_id_[dxl.second] = state_list_id;
    state_list_id_2motor_id_[state_list_id] = dxl.second;
    state_list_id += 1;
  }

  return true;
}

bool LoCoBotController::loadDynamixels(void) {
  bool result = false;
  const char *log;

  for (auto const &dxl:dynamixel_name_2ids_) {
    uint16_t model_number = 0;
    result = dxl_wb_->ping((uint8_t) dxl.second, &model_number, &log);
    if (!result) {
      ROS_ERROR("%s", log);
      ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
      return result;
    } else {
      ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
    }
  }
  return result;
}

bool LoCoBotController::initDynamixels(void) {
  const char *log;

  for (auto const &dxl:dynamixel_name_2ids_) {
    dxl_wb_->torqueOff((uint8_t) dxl.second);

    if(torque_control_ && (dxl.first == "joint_1" || dxl.first == "joint_2" || dxl.first == "joint_3" || dxl.first == "joint_4" || dxl.first == "joint2Dual")){
      dxl_wb_->itemWrite(dxl.second, "Operating_Mode", CURRENT_CONTROL_MODE, &log);
      usleep(10000);
    }
    else{
      dxl_wb_->itemWrite(dxl.second, "Operating_Mode", POSITION_CONTROL_MODE, &log);
      usleep(10000);
      for (auto const &info:dynamixel_info_) {
        if (dxl.first == info.first) {
          if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate") {
            bool
                result = dxl_wb_->itemWrite((uint8_t) dxl.second, info.second.item_name.c_str(), info.second.value, &log);
            if (!result) {
              ROS_ERROR("%s", log);
              ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]",
                        info.second.value,
                        info.second.item_name.c_str(),
                        dxl.first.c_str(),
                        dxl.second);
              return false;
            }
          }
        }
      }
  }
    dxl_wb_->torqueOn((uint8_t) dxl.second);
  }
  is_initialized_ = true;

  return true;
}

bool LoCoBotController::stopDynamixels(void) {
  const char *log;
  for (auto const &dxl:dynamixel_name_2ids_) {
    dxl_wb_->torqueOff((uint8_t) dxl.second);
  }
  return true;
}

bool LoCoBotController::initControlItems(void) {
  bool result = false;
  const char *log = NULL;

  auto it = dynamixel_name_2ids_.begin();

  const ControlItem *goal_position = dxl_wb_->getItemInfo(it->second,
                                                          "Goal_Position");
  if (goal_position == NULL) return false;

  const ControlItem *goal_velocity = dxl_wb_->getItemInfo(it->second,
                                                          "Goal_Velocity");
  if (goal_velocity == NULL)
    goal_velocity = dxl_wb_->getItemInfo(it->second,
                                         "Moving_Speed");
  if (goal_velocity == NULL) return false;

  const ControlItem *present_position = dxl_wb_->getItemInfo(it->second,
                                                             "Present_Position");
  if (present_position == NULL) return false;

  const ControlItem *present_velocity = dxl_wb_->getItemInfo(it->second,
                                                             "Present_Velocity");
  if (present_velocity == NULL)
    present_velocity = dxl_wb_->getItemInfo(it->second,
                                            "Present_Speed");
  if (present_velocity == NULL) return false;

  const ControlItem *present_current = dxl_wb_->getItemInfo(it->second,
                                                            "Present_Current");
  if (present_current == NULL)
    present_current = dxl_wb_->getItemInfo(it->second,
                                           "Present_Load");
  if (present_current == NULL) return false;

  const ControlItem *torque_enable = dxl_wb_->getItemInfo(it->second,
                                                          "Torque_Enable");
  if (torque_enable == NULL) return false;

  const ControlItem *shutdown = dxl_wb_->getItemInfo(it->second,
                                                          "Hardware_Error_Status");
  if (shutdown == NULL) return false;

  control_items_["Goal_Position"] = goal_position;
  control_items_["Goal_Velocity"] = goal_velocity;

  control_items_["Present_Position"] = present_position;
  control_items_["Present_Velocity"] = present_velocity;
  control_items_["Present_Current"] = present_current;

  control_items_["Torque_Enable"] = torque_enable;
  control_items_["Hardware_Error_Status"] = shutdown;


  return true;
}

bool LoCoBotController::initSDKHandlers(void) {
  bool result = false;

  const char *log = NULL;

//  auto it = dynamixel_name_2ids_.begin();

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address,
                                        control_items_["Goal_Position"]->data_length,
                                        &log);
  if (!result) {
    ROS_ERROR("%s", log);
    return result;
  } else {
    ROS_INFO("%s", log);
  }

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address,
                                        control_items_["Goal_Velocity"]->data_length,
                                        &log);
  if (!result) {
    ROS_ERROR("%s", log);
    return result;
  } else {
    ROS_INFO("%s", log);
  }

  if (dxl_wb_->getProtocolVersion() == 2.0f) {
    uint16_t start_address = std::min(control_items_["Present_Position"]->address,
                                      control_items_["Present_Current"]->address);
    uint16_t read_length = control_items_["Present_Position"]->data_length
        + control_items_["Present_Velocity"]->data_length
        + control_items_["Present_Current"]->data_length;
    result = dxl_wb_->addSyncReadHandler(start_address,
                                         read_length,
                                         &log);
    if (!result) {
      ROS_ERROR("%s", log);
      return result;
    }

    start_address = control_items_["Torque_Enable"]->address;
    read_length = control_items_["Torque_Enable"]->data_length;

    result = dxl_wb_->addSyncReadHandler(start_address,
                                         read_length,
                                         &log);
    if (!result) {
      ROS_ERROR("%s", log);
      return result;
    }

    start_address = control_items_["Hardware_Error_Status"]->address;
    read_length = control_items_["Hardware_Error_Status"]->data_length;

    result = dxl_wb_->addSyncReadHandler(start_address,
                                         read_length,
                                         &log);
    if (!result) {
      ROS_ERROR("%s", log);
      return result;
    }

  }

  num_motors_ = 0;
  for (auto const& group:use_group_)
  {
    if (group.second)
      num_motors_ += group_motors_[group.first].size();
  }

  return result;
}

void LoCoBotController::goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg) {

  if (msg->position.size() != 5) {
    ROS_ERROR("Invalid joint state, execution aborted");
  } else if (!use_group_["arm"]) {
    ROS_ERROR("Please initialize arm before commanding trajectories; set use_arm to true");
  } else {

    bool result = false;
    const char *log = NULL;

    uint8_t id_array[group_motors_["arm"].size()];
    uint8_t id_cnt = 0;

    int32_t dynamixel_position[group_motors_["arm"].size()];

    static uint32_t point_cnt = 0;
    static uint32_t position_cnt = 0;

    for (auto const &motor:group_motors_["arm"]) {
      id_array[id_cnt++] = dynamixel_name_2ids_[motor];
    }

    for (uint8_t index = 0; index < id_cnt; index++) {
      uint8_t id = id_array[index];
      if (id <= 2) {
        dynamixel_position[index] = dxl_wb_->convertRadian2Value(id, msg->position.at(id - 1));
      } else if (id == 3) {
        // Dual motor in joint 2, just reflect the position
        dynamixel_position[index] = dxl_wb_->convertRadian2Value(id, -msg->position.at(1));
      } else {
        dynamixel_position[index] = dxl_wb_->convertRadian2Value(id, -msg->position.at(id - 2));
      }
    }
    std::unique_lock<std::mutex> lock(dynamixel_mutex_);
    result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION,
                                id_array, id_cnt, dynamixel_position, 1, &log);
    if (!result) {
      ROS_ERROR("%s", log);
    }

  }
}

void LoCoBotController::execute_joint_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal) {

  ros::Time trajectoryStartTime = ros::Time::now();
  ros::Rate loop_rate(25);

  if (goal->trajectory.points.size() < 2) {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    smooth_joint_trajectory_server_->setSucceeded(result);
    return;
  }

  bool result = false;
  const char *log = NULL;

  uint8_t id_array[group_motors_["arm"].size()];
  uint8_t id_cnt = 0;

  int32_t dynamixel_position[group_motors_["arm"].size()];

  static uint32_t point_cnt = 0;
  static uint32_t position_cnt = 0;

  for (auto const &motor:group_motors_["arm"]) {
    id_array[id_cnt++] = dynamixel_name_2ids_[motor];
  }

  for (int i = 0; i < goal->trajectory.points.size(); i++) {
    //check for preempt requests from clients
    if (smooth_joint_trajectory_server_->isPreemptRequested()) {

      //preempt action server
      smooth_joint_trajectory_server_->setPreempted();
      ROS_INFO("Joint trajectory server preempted by client");
      return;
    }

    while ((ros::Time::now() - trajectoryStartTime).toSec() < goal->trajectory.points[i].time_from_start.toSec()) {
      continue;
    }

    for (uint8_t index = 0; index < id_cnt; index++) {
      uint8_t id = id_array[index];
      if (id <= 2) {
        dynamixel_position[index] =
            dxl_wb_->convertRadian2Value(id, goal->trajectory.points[i].positions[id - 1]);
      } else if (id == 3) {
        dynamixel_position[index] = dxl_wb_->convertRadian2Value(id,
                                                                 -goal->trajectory.points[i].positions[1]);
      } else {
        dynamixel_position[index] =
            dxl_wb_->convertRadian2Value(id, -goal->trajectory.points[i].positions[id - 2]);
      }
    }

    {
      std::unique_lock<std::mutex> lock(dynamixel_mutex_);
      result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION,
                                  id_array, id_cnt, dynamixel_position, 1, &log);
    }
    if (!result) {
      ROS_ERROR("%s", log);
    }

    loop_rate.sleep();

  }

  control_msgs::FollowJointTrajectoryResult actionlib_result;
  actionlib_result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
  smooth_joint_trajectory_server_->setSucceeded(actionlib_result);
  ROS_INFO("Trajectory goal execution took %fs.", (ros::Time::now() - trajectoryStartTime).toSec());
}

void LoCoBotController::initServer() {
  if (use_group_["arm"]) {
    if (torque_control_) {
      joint_command_server_ =
          node_handle_.advertiseService("torque_command", &LoCoBotController::jointCommandMsgCallback, this);
    } else {
      joint_command_server_ =
          node_handle_.advertiseService("joint_command", &LoCoBotController::jointCommandMsgCallback, this);
    }
  }
}

void LoCoBotController::initActionlib() {
  smooth_joint_trajectory_server_ = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(
      node_handle_,
      "locobot_arm/joint_controller/trajectory",
      boost::bind(&LoCoBotController::execute_joint_trajectory, this, _1),
      false);
  smooth_joint_trajectory_server_->start();
}

void LoCoBotController::initPublisher() {
  status_pub_ = node_handle_.advertise<std_msgs::Bool>("arm/status", 10);
  if (use_group_["camera"]) {
    pan_state_pub_ = node_handle_.advertise<std_msgs::Float64>("pan/state", 10);
    tilt_state_pub_ = node_handle_.advertise<std_msgs::Float64>("tilt/state", 10);
  }

  joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 100);
  gripper_state_pub_ = node_handle_.advertise<std_msgs::Int8>("gripper/state", 10);

}

void LoCoBotController::initSubscriber() {
  if (use_group_["arm"]) {
    joint_command_sub_ =
        node_handle_.subscribe("goal_dynamixel_position", 10,
                               &LoCoBotController::goalJointPositionCallback, this);
    stop_execution_sub_ = node_handle_.subscribe("stop_execution", 10,
                                                 &LoCoBotController::stopExecutionCallback, this);
    stop_execution_j4_sub_ =
        node_handle_.subscribe("stop_execution_j4", 10,
                               &LoCoBotController::stopExecutionJ4Callback, this);
    stop_execution_j5_sub_ =
        node_handle_.subscribe("stop_execution_j5", 10,
                               &LoCoBotController::stopExecutionJ5Callback, this);
  }

  if (use_group_["gripper"]) {
    gripper_close_sub_ = node_handle_.subscribe("gripper/close", 10,
                                                &LoCoBotController::gripperCloseCallback, this);
    gripper_open_sub_ = node_handle_.subscribe("gripper/open", 10,
                                               &LoCoBotController::gripperOpenCallback, this);
  }

  if (use_group_["camera"]) {
    set_pan_sub_ = node_handle_.subscribe("pan/command", 10,
                                          &LoCoBotController::setPanCallback, this);
    set_tilt_sub_ = node_handle_.subscribe("tilt/command", 10,
                                           &LoCoBotController::setTiltCallback, this);
  }

}

void LoCoBotController::gripperController(const ros::TimerEvent &) {
  bool result = false;
  const char *log = NULL;

  if (use_group_["gripper"]) {
    int gripper_motor_id = dynamixel_name_2ids_[group_motors_["gripper"][0]];
    int gripper_state_id = motor_id_2state_list_id_[gripper_motor_id];
    // close gripper
    if (gripper_state_ == 1) {
      if (prev_gripper_state_ != gripper_state_) {
        // shifting to PWM mode
        std::unique_lock<std::mutex> lock(dynamixel_mutex_);
        dxl_wb_->itemWrite(gripper_motor_id, "Torque_Enable", 0, &log);
        dxl_wb_->itemWrite(gripper_motor_id, "Operating_Mode", 16, &log);
        usleep(10000);
        dxl_wb_->itemWrite(gripper_motor_id, "Torque_Enable", 1, &log);
        prev_gripper_state_ = gripper_state_;
      }

      int32_t load = 0.0;
      load = (int32_t) dynamixel_state_list_.dynamixel_state[gripper_state_id].present_current;

      if (std::abs(load) > GRIPPER_MAX_LOAD) {
        // sometime gripper gets stuck.. this part is to overcome that
        if (prev_gripper_load_ == 0) {
          std::unique_lock<std::mutex> lock(dynamixel_mutex_);
          dxl_wb_->itemWrite(gripper_motor_id, "Goal_PWM", -700, &log);
          usleep(100000);
        }

        load = (int32_t) dynamixel_state_list_.dynamixel_state[gripper_state_id].present_current;

        if (std::abs(load) > GRIPPER_MAX_LOAD) {
          prev_gripper_load_ = 1;
        }
      }

      int32_t position = (int32_t) dynamixel_state_list_.dynamixel_state[gripper_state_id].present_position;
      if ((position < GRIPPER_CLOSE_VALUE)
          || prev_gripper_load_ == 1) {
        std::unique_lock<std::mutex> lock(dynamixel_mutex_);
        dxl_wb_->itemWrite(gripper_motor_id, "Goal_PWM", 0, &log);
        gripper_state_ = -1;
      } else {
        std::unique_lock<std::mutex> lock(dynamixel_mutex_);
        dxl_wb_->itemWrite(gripper_motor_id, "Goal_PWM", -GRIPPER_PWM, &log);
      }

    }
      // open gripper
    else if (gripper_state_ == 0) {

      std::unique_lock<std::mutex> lock(dynamixel_mutex_);
      if (prev_gripper_state_ != gripper_state_) {
        // shifting to position mode
        dxl_wb_->itemWrite(gripper_motor_id, "Torque_Enable", 0, &log);
        dxl_wb_->itemWrite(gripper_motor_id, "Operating_Mode", 3, &log);
        usleep(10000);
        dxl_wb_->itemWrite(gripper_motor_id, "Torque_Enable", 1, &log);
        prev_gripper_state_ = gripper_state_;
      }
      dxl_wb_->goalPosition(gripper_motor_id, GRIPPER_OPEN_VALUE);
      prev_gripper_load_ = 0;

    gripper_state_ = -1;
    }
  int32_t position = (int32_t) dynamixel_state_list_.dynamixel_state[gripper_state_id].present_position;
  int32_t delta = 50;
  // publish state
  if(gripper_cmd_ == -1 or gripper_cmd_ == 0){
    if(position > GRIPPER_OPEN_VALUE - delta && position < GRIPPER_OPEN_VALUE + delta){
      // fully open 0
      gripper_state_msg_.data = 0;
      }
    else{
      // unknown -1
      gripper_state_msg_.data = -1;
      }
  }
  else if(gripper_cmd_ == 1){
   if(gripper_state_ != -1){
      // closing
      gripper_state_msg_.data = 1;
      }
   else if(position < GRIPPER_CLOSE_VALUE){
      // fully closed 2
      gripper_state_msg_.data = 3;
      }
   else{
      // have object in hand 1
      gripper_state_msg_.data = 2;
      }
    }
    gripper_state_pub_.publish(gripper_state_msg_);
  }

}

bool LoCoBotController::controlLoop(void) {
  // hardwareStatusPrint(); Enabling this function will severly slow down the node
  return hardwareOK();
};


bool LoCoBotController::hardwareStatusPrint() {

  for (auto const& group:use_group_) {
    if (group.second) {

      for (auto const &motor:group_motors_[group.first]) {
        int motor_id = dynamixel_name_2ids_[motor];
        const char *read_log;
        int32_t position = 0;
        dxl_wb_->itemRead(motor_id, "Current_Limit", &position, &read_log);
        ROS_INFO("Motor ID %d current limit %d\n", motor_id, position);

        position = 0;
        dxl_wb_->itemRead(motor_id, "Max_Voltage_Limit", &position, &read_log);
        ROS_INFO("Motor ID %d Max Voltage Limit  %d\n", motor_id, position);

        position = 0;
        dxl_wb_->itemRead(motor_id, "Min_Voltage_Limit", &position, &read_log);
        ROS_INFO("Motor ID %d Min Voltage Limit  %d\n", motor_id, position);

        position = 0;
        dxl_wb_->itemRead(motor_id, "Temperature_Limit", &position, &read_log);
        ROS_INFO("Motor ID %d Temperature Limit  %d\n", motor_id, position);

        position = 0;
        dxl_wb_->itemRead(motor_id, "Present_Temperature", &position, &read_log);
        ROS_INFO("Motor ID %d Present  Temperature %d\n", motor_id, position);

        position = 0;
        dxl_wb_->itemRead(motor_id, "Present_Input_Voltage", &position, &read_log);
        ROS_INFO("Motor ID %d Present Input voltage %d\n", motor_id, position);

        position = 0;
        dxl_wb_->itemRead(motor_id, "Present_Current", &position, &read_log);
        ROS_INFO("Motor ID %d Present Current %d\n", motor_id, position);

        position = 0;
        dxl_wb_->itemRead(motor_id, "Present_PWM", &position, &read_log);
        ROS_INFO("Motor ID %d Present PWM %d\n", motor_id, position);

      }

    }
  }

  return true;
}

void LoCoBotController::readCallback(const ros::TimerEvent &) {
#ifdef DEBUG
  static double priv_read_secs =ros::Time::now().toSec();
#endif
  bool result = false;

  const char *log = NULL;

  dynamixel_workbench_msgs::DynamixelState dynamixel_state[dynamixel_name_2ids_.size()];
  dynamixel_state_list_.dynamixel_state.clear();

  int32_t get_current[dynamixel_name_2ids_.size()];
  int32_t get_velocity[dynamixel_name_2ids_.size()];
  int32_t get_position[dynamixel_name_2ids_.size()];

  uint8_t id_array[dynamixel_name_2ids_.size()];
  uint8_t id_cnt = 0;

  {

    for (auto const &dxl:dynamixel_name_2ids_) {
      dynamixel_state[id_cnt].name = dxl.first;
      dynamixel_state[id_cnt].id = (uint8_t) dxl.second;

      id_array[id_cnt++] = (uint8_t) dxl.second;
    }

    if (dxl_wb_->getProtocolVersion() == 2.0f) {
      std::unique_lock<std::mutex> lock(dynamixel_mutex_);
      result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                 id_array,
                                 dynamixel_name_2ids_.size(),
                                 &log);
      if (!result) {
        ROS_ERROR("%s", log);
      }

      result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                        id_array,
                                        id_cnt,
                                        control_items_["Present_Current"]->address,
                                        control_items_["Present_Current"]->data_length,
                                        get_current,
                                        &log);
      if (!result) {
        ROS_ERROR("%s", log);
      }

      result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                        id_array,
                                        id_cnt,
                                        control_items_["Present_Velocity"]->address,
                                        control_items_["Present_Velocity"]->data_length,
                                        get_velocity,
                                        &log);
      if (!result) {
        ROS_ERROR("%s", log);
      }

      result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                        id_array,
                                        id_cnt,
                                        control_items_["Present_Position"]->address,
                                        control_items_["Present_Position"]->data_length,
                                        get_position,
                                        &log);
      if (!result) {
        ROS_ERROR("%s", log);
      }

      for (uint8_t index = 0; index < id_cnt; index++) {
        dynamixel_state[index].present_current = get_current[index];
        dynamixel_state[index].present_velocity = get_velocity[index];
        dynamixel_state[index].present_position = get_position[index];

        dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[index]);
      }
    } else if (dxl_wb_->getProtocolVersion() == 1.0f) {
      std::unique_lock<std::mutex> lock(dynamixel_mutex_);
      uint16_t length_of_data = control_items_["Present_Position"]->data_length +
          control_items_["Present_Velocity"]->data_length +
          control_items_["Present_Current"]->data_length;
      uint32_t get_all_data[length_of_data];
      uint8_t dxl_cnt = 0;
      for (auto const &dxl:dynamixel_name_2ids_) {
        result = dxl_wb_->readRegister((uint8_t) dxl.second,
                                       control_items_["Present_Position"]->address,
                                       length_of_data,
                                       get_all_data,
                                       &log);
        if (!result) {
          ROS_ERROR("%s", log);
        }

        dynamixel_state[dxl_cnt].present_current = DXL_MAKEWORD(get_all_data[4], get_all_data[5]);
        dynamixel_state[dxl_cnt].present_velocity = DXL_MAKEWORD(get_all_data[2], get_all_data[3]);
        dynamixel_state[dxl_cnt].present_position = DXL_MAKEWORD(get_all_data[0], get_all_data[1]);

        dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[dxl_cnt]);
        dxl_cnt++;
      }
    }
  }

#ifdef DEBUG
  ROS_WARN("[readCallback] diff_secs : %f", ros::Time::now().toSec() - priv_read_secs);
  priv_read_secs = ros::Time::now().toSec();
#endif
}

void LoCoBotController::publishCallback(const ros::TimerEvent &) {
#ifdef DEBUG
  static double priv_pub_secs =ros::Time::now().toSec();
#endif

  joint_state_msg_.header.stamp = ros::Time::now();

  joint_state_msg_.name.clear();
  joint_state_msg_.position.clear();
  joint_state_msg_.velocity.clear();
  joint_state_msg_.effort.clear();

  std::stringstream joint_name;

  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;

  std::string arm_joint_names[5] = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5"};

  // STEP 1 - Populate joint_state message with arm motor readings
  if (use_group_["arm"]) {
    for (int i = 0; i < 5; i++) {
      std::string motor = arm_joint_names[i];
      int motor_id = dynamixel_name_2ids_[motor];
      int state_id = motor_id_2state_list_id_[motor_id];
      if (motor_id == 3) {
        // Shadow motor for joint 2 - ignore
        continue;
      } else {
        joint_state_msg_.name.push_back(motor);

        if (dxl_wb_->getProtocolVersion() == 2.0f) {
          if (strcmp(dxl_wb_->getModelName((uint8_t) motor_id), "XL-320") == 0)
            effort =
                dxl_wb_->convertValue2Load((int16_t) dynamixel_state_list_.dynamixel_state[state_id].present_current);
          else
            effort = dxl_wb_
                ->convertValue2Current((int16_t) dynamixel_state_list_.dynamixel_state[state_id].present_current);
        } else if (dxl_wb_->getProtocolVersion() == 1.0f)
          effort = dxl_wb_
              ->convertValue2Load((int16_t) dynamixel_state_list_.dynamixel_state[state_id].present_current);

        velocity = dxl_wb_->convertValue2Velocity((uint8_t) motor_id,
                                                  (int32_t) dynamixel_state_list_.dynamixel_state[state_id]
                                                      .present_velocity);
        position = dxl_wb_->convertValue2Radian((uint8_t) motor_id,
                                                (int32_t) dynamixel_state_list_.dynamixel_state[state_id]
                                                    .present_position);

        if ((uint8_t) motor_id >= 4) {
          velocity = -velocity;
          position = -position;
        }
        joint_state_msg_.effort.push_back(effort);
        joint_state_msg_.velocity.push_back(velocity);
        joint_state_msg_.position.push_back(position);
      }
    }

  } else {
    for (int index = 1; index <= 5; index++) {
      joint_name.str("");
      joint_name << "joint_" << index;

      joint_state_msg_.name.push_back(joint_name.str());
      joint_state_msg_.effort.push_back(effort);
      joint_state_msg_.velocity.push_back(velocity);
      joint_state_msg_.position.push_back(position);
    }
  }

  // STEP 2 - Populate joint_state message with gripper readings

  position = 0.0;
  velocity = 0.0;
  effort = 0.0;
  if (use_group_["gripper"]) {
    int motor_id = dynamixel_name_2ids_[group_motors_["gripper"][0]];
    int state_id = motor_id_2state_list_id_[motor_id];
    if (dxl_wb_->getProtocolVersion() == 2.0f) {
      if (strcmp(dxl_wb_->getModelName((uint8_t) motor_id), "XL-320") == 0)
        effort =
            dxl_wb_->convertValue2Load((int16_t) dynamixel_state_list_.dynamixel_state[state_id].present_current);
      else
        effort = dxl_wb_
            ->convertValue2Current((int16_t) dynamixel_state_list_.dynamixel_state[state_id].present_current);
    } else if (dxl_wb_->getProtocolVersion() == 1.0f)
      effort = dxl_wb_
          ->convertValue2Load((int16_t) dynamixel_state_list_.dynamixel_state[state_id].present_current);

    velocity = dxl_wb_->convertValue2Velocity((uint8_t) motor_id,
                                              (int32_t) dynamixel_state_list_.dynamixel_state[state_id]
                                                  .present_velocity);
    position = dxl_wb_->convertValue2Radian((uint8_t) motor_id,
                                            (int32_t) dynamixel_state_list_.dynamixel_state[state_id]
                                                .present_position);
    position = (-position - GRIPPER_OPEN_MOTOR_POS)/(GRIPPER_CLOSE_MOTOR_POS - 
      GRIPPER_OPEN_MOTOR_POS) * (GRIPPER_CLOSE_MOVEIT - GRIPPER_OPEN_MOVEIT) + GRIPPER_OPEN_MOVEIT;
  } else {
    position = GRIPPER_OPEN_MOVEIT;
  }
  joint_name.str("");
  joint_name << "joint_6";
  joint_state_msg_.name.push_back(joint_name.str());
  joint_state_msg_.effort.push_back(effort);
  joint_state_msg_.velocity.push_back(velocity);
  joint_state_msg_.position.push_back(position);

  joint_name.str("");
  joint_name << "joint_7";
  joint_state_msg_.name.push_back(joint_name.str());
  joint_state_msg_.effort.push_back(effort);
  joint_state_msg_.velocity.push_back(-velocity);
  joint_state_msg_.position.push_back(-position);

  // STEP 3 - Populate joint_state message with camera readings
  position = 0.0;
  velocity = 0.0;
  effort = 0.0;

  if (use_group_["camera"]) {
    for (auto const &motor:group_motors_["camera"]) {
      int motor_id = dynamixel_name_2ids_[motor];
      int state_id = motor_id_2state_list_id_[motor_id];
      joint_state_msg_.name.push_back(motor);

      if (dxl_wb_->getProtocolVersion() == 2.0f) {
        if (strcmp(dxl_wb_->getModelName((uint8_t) motor_id), "XL-320") == 0)
          effort = dxl_wb_
              ->convertValue2Load((int16_t) dynamixel_state_list_.dynamixel_state[state_id].present_current);
        else
          effort = dxl_wb_
              ->convertValue2Current((int16_t) dynamixel_state_list_.dynamixel_state[state_id].present_current);
      } else if (dxl_wb_->getProtocolVersion() == 1.0f)
        effort = dxl_wb_
            ->convertValue2Load((int16_t) dynamixel_state_list_.dynamixel_state[state_id].present_current);

      velocity = dxl_wb_->convertValue2Velocity((uint8_t) motor_id,
                                                (int32_t) dynamixel_state_list_
                                                    .dynamixel_state[state_id].present_velocity);
      position = dxl_wb_->convertValue2Radian((uint8_t) motor_id,
                                              (int32_t) dynamixel_state_list_
                                                  .dynamixel_state[state_id].present_position);

      joint_state_msg_.effort.push_back(effort);
      joint_state_msg_.velocity.push_back(velocity);
      joint_state_msg_.position.push_back(position);
    }
  } else {
    joint_name.str("");
    joint_name << "head_pan_joint";
    joint_state_msg_.name.push_back(joint_name.str());
    joint_state_msg_.effort.push_back(effort);
    joint_state_msg_.velocity.push_back(velocity);
    joint_state_msg_.position.push_back(position);

    joint_name.str("");
    joint_name << "head_tilt_joint";
    joint_state_msg_.name.push_back(joint_name.str());
    joint_state_msg_.effort.push_back(effort);
    joint_state_msg_.velocity.push_back(velocity);
    joint_state_msg_.position.push_back(position);

  }

  joint_states_pub_.publish(joint_state_msg_);
  if (use_group_["camera"]) {
    cameraStatePublish();
  }

#ifdef DEBUG
  ROS_WARN("[publishCallback] diff_secs : %f", ros::Time::now().toSec() - priv_pub_secs);
  priv_pub_secs = ros::Time::now().toSec();
#endif
}

bool LoCoBotController::jointCommandMsgCallback(locobot_control::JointCommand::Request &req,
                                                locobot_control::JointCommand::Response &res) {
  /*1 to 5 would be each arm joint, 6 for gripper and 7,8 for pan and tilt respectively*/

  int32_t goal_position = 0;
  bool ret;
  const char *log;

  if (req.id == 1) {
    if (torque_control_) {
      if (req.unit == "newt") {
        goal_position = int32_t(req.goal_position*torque_to_current_value_ratio);
      } else {
        goal_position = int32_t(req.goal_position);
      }
      ret = dxl_wb_->itemWrite(req.id, "Goal_Current", goal_position, &log);
    } else {
      if (req.unit == "rad") {
        goal_position = dxl_wb_->convertRadian2Value(req.id, req.goal_position);
      } else {
        goal_position = req.goal_position;
      }

      ret = dxl_wb_->goalPosition(req.id, goal_position, &log);
    }

  } else if (req.id == 2) {
    if (torque_control_) {
      if (req.unit == "newt") {
        goal_position = int32_t(req.goal_position*torque_to_current_value_ratio);
      } else {
        goal_position = req.goal_position;
      }
      goal_position = goal_position / 2;         // as there are 2 motors
      ret = dxl_wb_->itemWrite(req.id, "Goal_Current", goal_position, &log);

      ret = ret && dxl_wb_->itemWrite(req.id + 1, "Goal_Current", -1 * goal_position, &log);
    } else {
      if (req.unit == "rad") {
        goal_position = dxl_wb_->convertRadian2Value(req.id, req.goal_position);
      } else {
        goal_position = req.goal_position;
      }

      ret = dxl_wb_->goalPosition(req.id, goal_position, &log);

      // For the other motor operating for the same joint
      if (req.unit == "rad") {
        goal_position = dxl_wb_->convertRadian2Value(req.id, -1 * req.goal_position);
      } else {
        goal_position = -1 * req.goal_position;
      }

      ret = ret && dxl_wb_->goalPosition(req.id + 1, goal_position, &log);
    }

  } else if ((req.id >= 3) && (req.id < 5)) {
    if (torque_control_) {
      if (req.unit == "newt") {
        goal_position = -1 * int32_t(req.goal_position*torque_to_current_value_ratio);
      } else {
        goal_position = -1 * req.goal_position;
      }

      ret = dxl_wb_->itemWrite(req.id + 1, "Goal_Current", goal_position, &log);
    } else {
      if (req.unit == "rad") {
        goal_position = dxl_wb_->convertRadian2Value(req.id + 1, -1 * req.goal_position);
      } else {
        goal_position = -1 * req.goal_position;
      }

      ret = dxl_wb_->goalPosition(req.id + 1, goal_position, &log);
    }
  } else if (req.id == 5) {
    if (torque_control_) {
      if (req.unit == "newt") {
        ROS_INFO("This motor does not support direct torque... You can write PWM[which is voltage control]");
        ret = false;
      } else {
        goal_position = -1 * req.goal_position;
      }

      ret = dxl_wb_->itemWrite(req.id + 1, "Goal_PWM", goal_position, &log);
    } else {
      if (req.unit == "rad") {
        goal_position = dxl_wb_->convertRadian2Value(req.id + 1, -1 * req.goal_position);
      } else {
        goal_position = -1 * req.goal_position;
      }

      ret = dxl_wb_->goalPosition(req.id + 1, goal_position, &log);
    }
  } else {
    ROS_INFO("Invalid joint id %d", req.id);
    ret = false;
  }

  res.result = ret;
}

void LoCoBotController::stopExecutionCallback(const std_msgs::Empty::ConstPtr &msg) {
  ROS_INFO("Stopping execution");
  bool result = false;
  const char *log = NULL;

  uint8_t id_cnt = dynamixel_name_2ids_.size();
  uint8_t id_array[id_cnt];
  uint8_t index = 0;

  int32_t dynamixel_position[id_cnt];

  for (auto const &dxl:dynamixel_name_2ids_) {
    id_array[index] = dxl.second;
    dynamixel_position[index] = dynamixel_state_list_.dynamixel_state[index].present_position;
    index++;
  }
  std::unique_lock<std::mutex> lock(dynamixel_mutex_);
  result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION,
                              id_array, id_cnt, dynamixel_position, 1, &log);
  if (!result) {
    ROS_ERROR("%s", log);
  }

}

void LoCoBotController::stopExecutionJ4Callback(const std_msgs::Empty::ConstPtr &msg) {
  ROS_INFO("Stopping execution for joint 4");
  if (use_group_["arm"]) {
    bool result = false;
    const char *read_log;
    const char *write_log;
    int32_t position;
    result = dxl_wb_->itemRead(5, "Present_Position", &position, &read_log);
    if (result) {
      result = dxl_wb_->goalPosition(5, position, &write_log);
      if (!result) {
        ROS_ERROR("%s", write_log);
      }
    } else {
      ROS_ERROR("%s", read_log);
    }
  } else {
    ROS_ERROR("Arm is not initialized, make sure use_arm:=True and arm motors are connected");
  }
}

void LoCoBotController::stopExecutionJ5Callback(const std_msgs::Empty::ConstPtr &msg) {
  ROS_INFO("Stopping execution for joint 5");
  if (use_group_["arm"]) {
    bool result = false;
    const char *read_log;
    const char *write_log;
    int32_t position;
    result = dxl_wb_->itemRead(6, "Present_Position", &position, &read_log);
    if (result) {
      result = dxl_wb_->goalPosition(6, position, &write_log);
      if (!result) {
        ROS_ERROR("%s", write_log);
      }
    } else {
      ROS_ERROR("%s", read_log);
    }
  } else {
    ROS_ERROR("Arm is not initialized, make sure use_arm:=True and arm motors are connected");
  }
}

void LoCoBotController::gripperCloseCallback(const std_msgs::Empty::ConstPtr &msg) {
  ROS_INFO("Gripper close");
  gripper_state_ = 1;
  gripper_cmd_ = 1;
}

void LoCoBotController::gripperOpenCallback(const std_msgs::Empty::ConstPtr &msg) {
  ROS_INFO("Gripper open");
  const char *log;
  dxl_wb_->itemWrite(7, "Goal_PWM", GRIPPER_PWM, &log);
  gripper_state_ = 0;
  gripper_cmd_ = 0;
}

void LoCoBotController::setPanCallback(const std_msgs::Float64::ConstPtr &msg) {
  ROS_INFO("Set camera pan");
  if (use_group_["camera"]) {
    bool result = false;
    const char *log;
    result = dxl_wb_->goalPosition(8, dxl_wb_->convertRadian2Value(8, msg->data), &log);
    if (!result) {
      ROS_ERROR("%s", log);
    }
  } else {
    ROS_ERROR("Pan-tilt Camera is not initialized, make sure use_camera:=True and camera motors are connected");
  }
}

void LoCoBotController::setTiltCallback(const std_msgs::Float64::ConstPtr &msg) {
  ROS_INFO("Set camera tilt");
  if (use_group_["camera"]) {
    bool result = false;
    const char *log;
    result = dxl_wb_->goalPosition(9, dxl_wb_->convertRadian2Value(9, msg->data), &log);
    if (!result) {
      ROS_ERROR("%s", log);
    }
  } else {
    ROS_ERROR("Pan-tilt Camera is not initialized, make sure use_camera:=True and camera motors are connected");
  }
}

void LoCoBotController::cameraStatePublish(void) {
  std_msgs::Float64 msg;
  double position_pan = 0.0;
  double position_tilt = 0.0;

  if (use_group_["camera"]) {
    for (auto const &motor:group_motors_["camera"]) {
      int motor_id = dynamixel_name_2ids_[motor];
      int state_id = motor_id_2state_list_id_[motor_id];
      if (motor_id == 8) {
        position_pan = dxl_wb_->convertValue2Radian((uint8_t) motor_id,
                                                    (int32_t) dynamixel_state_list_
                                                        .dynamixel_state[state_id].present_position);
      } else if (motor_id == 9) {
        position_tilt = dxl_wb_->convertValue2Radian((uint8_t) motor_id,
                                                     (int32_t) dynamixel_state_list_
                                                         .dynamixel_state[state_id].present_position);
      }
    }
  }

  // Publish camera pan state
  msg.data = position_pan;
  pan_state_pub_.publish(msg);

  // Publish camera tilt state
  msg.data = position_tilt;
  tilt_state_pub_.publish(msg);
}

bool LoCoBotController::hardwareOK() {
  bool hardwareOK = true;
  std::unique_lock<std::mutex> lock(hardware_mutex_);
  hardwareOK = is_hardware_ok_;
  return hardwareOK;
}


void LoCoBotController::hardwareStatusPublish(const ros::TimerEvent &) {
  std_msgs::Bool status;
  status.data = true;
  bool result = false;
  const char *log = NULL;

  uint8_t id_array[num_motors_];
  uint8_t id_cnt = 0;
  int32_t shutdownInfo[num_motors_];
  std::map<int, std::string> array_id2motor_name;

  for (auto const& group:use_group_)
  {
    if (group.second) {
      for (auto const &motor:group_motors_[group.first]) {
        array_id2motor_name[id_cnt] = motor;
        id_array[id_cnt++] = (uint8_t) dynamixel_name_2ids_[motor];
      }
    }
  }

  if (dxl_wb_->getProtocolVersion() == 2.0f) {
    std::unique_lock<std::mutex> lock(dynamixel_mutex_);
    result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_SHUTDOWN,
                               id_array,
                               num_motors_,
                               &log);
    if (!result) {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_SHUTDOWN,
                                      id_array,
                                      id_cnt,
                                      control_items_["Hardware_Error_Status"]->address,
                                      control_items_["Hardware_Error_Status"]->data_length,
                                      shutdownInfo,
                                      &log);

    if (!result) {
      ROS_ERROR("%s", log);
    }
  } else if (dxl_wb_->getProtocolVersion() == 1.0f) {
    std::unique_lock<std::mutex> lock(dynamixel_mutex_);
    int idx = 0;
    for (int i = 0; i < id_cnt; i++) {
      uint32_t torque_enable_info[control_items_["Shutdown"]->data_length];
      result = dxl_wb_->readRegister((uint8_t) id_array[i],
                                     control_items_["Hardware_Error_Status"]->address,
                                     control_items_["Hardware_Error_Status"]->data_length,
                                     torque_enable_info,
                                     &log);
      if (!result) {
        ROS_ERROR("%s", log);
      }
      shutdownInfo[idx] = torque_enable_info[0];
    }
  }

  int32_t byte_info = 0;
  bool has_overload_error = false;
  bool has_electrical_error = false;
  bool has_encoder_error = false;
  bool has_overheating_error = false;
  bool has_input_voltage_error = false;

  for (uint8_t index = 0; index < id_cnt; index++) {
    byte_info = shutdownInfo[index];

    if (byte_info != 0) {

      status.data = false;
      status_pub_.publish(status);

      ROS_ERROR("Motor %s ID=%d has Hardware_Error_Status byte %d",
                array_id2motor_name[index].c_str(), (uint8_t) id_array[index], byte_info);

      has_overload_error = (bool) getBit(byte_info, HARDWARE_ERROR_STATUS_OVERLOAD);
      if (has_overload_error) {
        ROS_ERROR("Motor %s ID=%d has Overload error", array_id2motor_name[index].c_str(), (uint8_t) id_array[index]);
      }

      has_electrical_error = (bool) getBit(byte_info, HARDWARE_ERROR_STATUS_ELECTRICAL_SHOCK);
      if (has_electrical_error) {
        ROS_ERROR("Motor %s ID=%d has Electrical Shock error", array_id2motor_name[index].c_str(), (uint8_t) id_array[index]);
      }

      has_encoder_error = (bool) getBit(byte_info, HARDWARE_ERROR_STATUS_ENCODER);
      if (has_encoder_error) {
        ROS_ERROR("Motor %s ID=%d has Motor Encoder error", array_id2motor_name[index].c_str(), (uint8_t) id_array[index]);
      }

      has_overheating_error = (bool) getBit(byte_info, HARDWARE_ERROR_STATUS_OVERHEATING);
      if (has_overheating_error) {
        ROS_ERROR("Motor %s ID=%d has Overheating error", array_id2motor_name[index].c_str(), (uint8_t) id_array[index]);
      }

      has_input_voltage_error = (bool) getBit(byte_info, HARDWARE_ERROR_STATUS_INPUT_VOLTAGE);
      if (has_input_voltage_error) {
        ROS_ERROR("Motor %s ID=%d has Input Volatage error", array_id2motor_name[index].c_str(), (uint8_t) id_array[index]);
      }

      std::unique_lock<std::mutex> lock(hardware_mutex_);
      is_hardware_ok_ = false;

    }
  }

  status_pub_.publish(status);
}

int LoCoBotController::getBit(int n, int k) {
  return (n & (1<<k)) != 0;
}

bool LoCoBotController::dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                                    dynamixel_workbench_msgs::DynamixelCommand::Response &res) {
  bool result = false;
  const char *log;

  uint8_t id = req.id;
  std::string item_name = req.addr_name;
  int32_t value = req.value;

  result = dxl_wb_->itemWrite(id, item_name.c_str(), value, &log);
  if (!result) {
    ROS_ERROR("%s", log);
    ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[ID : %d]", value, item_name.c_str(), id);
  }

  res.comm_result = result;

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "locobot_controller");
  ros::NodeHandle node_handle("");

  std::string port_name = "/dev/ttyUSB0";
  uint32_t baud_rate = 57600;

  if (argc < 2) {
    ROS_ERROR("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
    return 0;
  } else {
    port_name = argv[1];
    baud_rate = atoi(argv[2]);
  }

  LoCoBotController dynamixel_controller;

  bool result = false;

  std::string yaml_file = node_handle.param<std::string>("dynamixel_info", "");

  result = dynamixel_controller.initWorkbench(port_name, baud_rate);
  if (!result) {
    ROS_ERROR("Please check USB port name");
    return 0;
  }

  result = dynamixel_controller.getDynamixelsInfo(yaml_file);
  if (!result) {
    ROS_ERROR("Please check YAML file");
    return 0;
  }

  result = dynamixel_controller.loadDynamixels();
  if (!result) {
    ROS_ERROR("Please check Dynamixel ID or BaudRate");
    return 0;
  }

  result = dynamixel_controller.initDynamixels();
  if (!result) {
    ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
    return 0;
  }

  result = dynamixel_controller.initControlItems();
  if (!result) {
    ROS_ERROR("Please check control items");
    return 0;
  }

  result = dynamixel_controller.initSDKHandlers();
  if (!result) {
    ROS_ERROR("Failed to set Dynamixel SDK Handler");
    return 0;
  }

  dynamixel_controller.initPublisher();
  dynamixel_controller.initSubscriber();
  dynamixel_controller.initServer();
  dynamixel_controller.initActionlib();

  ros::Timer read_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getReadPeriod()),
                                                  &LoCoBotController::readCallback, &dynamixel_controller);
  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getPublishPeriod()),
                                                     &LoCoBotController::publishCallback, &dynamixel_controller);
  ros::Timer gripper_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getPublishPeriod()),
                                                     &LoCoBotController::gripperController, &dynamixel_controller);
  ros::Timer hardware_timer2 = node_handle.createTimer(ros::Duration(dynamixel_controller.getPublishPeriod()),
                                                      &LoCoBotController::hardwareStatusPublish, &dynamixel_controller);
  bool status;

  ros::Rate loop_rate(50);

  while (ros::ok()) {
    status = dynamixel_controller.controlLoop();
    if (not status) {
      break;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
