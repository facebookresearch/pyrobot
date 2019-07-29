/*******************************************************************************
# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include<locobot_gazebo/gazebo_locobot_control.h>

bool isClose(const double a, const double b)
{
    if (abs(a-b) > 0.005) return false;
    return true;
}

GazeboInteface::GazeboInteface(): node_handle_("") {
    joint_1_pub = node_handle_.advertise < std_msgs::Float64 > ("/joint_1_cntrl/command", 10);
    joint_2_pub = node_handle_.advertise < std_msgs::Float64 > ("/joint_2_cntrl/command", 10);
    joint_3_pub = node_handle_.advertise < std_msgs::Float64 > ("/joint_3_cntrl/command", 10);
    joint_4_pub = node_handle_.advertise < std_msgs::Float64 > ("/joint_4_cntrl/command", 10);
    joint_5_pub = node_handle_.advertise < std_msgs::Float64 > ("/joint_5_cntrl/command", 10);
    joint_6_pub = node_handle_.advertise < std_msgs::Float64 > ("/joint_6_cntrl/command", 10);
    joint_7_pub = node_handle_.advertise < std_msgs::Float64 > ("/joint_7_cntrl/command", 10);
    head_pan_pub = node_handle_.advertise < std_msgs::Float64 > ("/pan/command", 10);
    head_tilt_pub = node_handle_.advertise < std_msgs::Float64 > ("/tilt/command", 10);

    node_handle_.getParam("torque_control", torque_control_);

    smooth_joint_trajectory_server_
        = new actionlib::SimpleActionServer < control_msgs::FollowJointTrajectoryAction > (
            node_handle_,
            "locobot_arm/joint_controller/trajectory",
            boost::bind( & GazeboInteface::executeJointTrajectory, this, _1),
            false);

    joint_command_sub_ = node_handle_.subscribe("goal_dynamixel_position", 10, &
        GazeboInteface::goalJointPositionCallback,
        this);

    gripper_open_sub_ = node_handle_.subscribe("gripper/open", 10, & GazeboInteface::gripperOpen, this);
    gripper_close_sub_ = node_handle_.subscribe("gripper/close", 10, & GazeboInteface::gripperClose, this);

    stop_joints_sub_ = node_handle_.subscribe("stop_execution", 10, & GazeboInteface::stopExecution, this);

    joint_state_sub_ = node_handle_.subscribe("joint_states", 10, & GazeboInteface::recordArmJoints, this);

    gripper_state_pub = node_handle_.advertise<std_msgs::Int8>("gripper/state", 10);


    pub_arr[0] = joint_1_pub;
    pub_arr[1] = joint_2_pub;
    pub_arr[2] = joint_3_pub;
    pub_arr[3] = joint_4_pub;
    pub_arr[4] = joint_5_pub;
    pub_arr[5] = joint_6_pub;
    pub_arr[6] = joint_7_pub;
    pub_arr[7] = head_pan_pub;
    pub_arr[8] = head_tilt_pub;

    smooth_joint_trajectory_server_->start();

    //individual joint command service!!

    joint_command_server_ = node_handle_.advertiseService("joint_command", &
        GazeboInteface::jointCommandMsgCallback, this);

    joint_torque_server_ = node_handle_.advertiseService("torque_command", 
                            &GazeboInteface::jointTorqueCommandMsgCallback, this);

    torque_command_client_ =  node_handle_.serviceClient<gazebo_msgs::ApplyJointEffort>(
                                                                    "/gazebo/apply_joint_effort");


    //set intial positions to 0 for all the joints
    ros::Duration(5).sleep();
    if (torque_control_)
    {
        // command 2 positon for joints 5,6,7, pan and tilt
        for (int index = 4; index < 9; index++) {
            std_msgs::Float64 msg;
            msg.data = 0;
            pub_arr[index].publish(msg);
        }

        // command zero torques for the first four joints
        for (int i = 1; i <5; ++i)
            SetJointTorque(i, 0);

        // command toruqes to hold th initial position
        double zero_postion_torques[4] = {-0.41155806170420917, -0.8730301010793617, 
                                            -0.8806836788762595, -0.1379062563443867};
        for (int i = 1; i <5; ++i)
            SetJointTorque(i, zero_postion_torques[i]);

    }
    else
    {
        for (int index = 0; index < 9; index++) {
            std_msgs::Float64 msg;
            msg.data = 0;
            pub_arr[index].publish(msg);
        }        
    }

    gripper_state = -1; // unknown
    gripper_timer = node_handle_.createTimer(ros::Duration(0.1),
                                                     &GazeboInteface::gripperStateCallback, this);
}


bool GazeboInteface::jointCommandMsgCallback(locobot_control::JointCommand::Request & req,
    locobot_control::JointCommand::Response & res) {
    // passed ids are from 1 to 5
    std_msgs::Float64 msg;
    msg.data = req.goal_position;

    pub_arr[req.id - 1].publish(msg);
    res.result = true;
}


void GazeboInteface::SetJointTorque(const int id, const double value)
{
     // When torque control for the first four motors
    gazebo_msgs::ApplyJointEffort effort_cmd_srv_msg;
    ros::Duration duration(-1);
    effort_cmd_srv_msg.request.duration = duration;
    effort_cmd_srv_msg.request.effort = value;
    effort_cmd_srv_msg.request.start_time = ros::Time::now();
    switch(id)
    {
        case 1: effort_cmd_srv_msg.request.joint_name = "joint_1";
            break;
        case 2: effort_cmd_srv_msg.request.joint_name = "joint_2";
            break;
        case 3: effort_cmd_srv_msg.request.joint_name = "joint_3";
            break;
        case 4: effort_cmd_srv_msg.request.joint_name = "joint_4";
            break;
    }

    torque_command_client_.call(effort_cmd_srv_msg);
}

bool GazeboInteface::jointTorqueCommandMsgCallback(locobot_control::JointCommand::Request &req,
                                              locobot_control::JointCommand::Response &res)
{
    // Torque control only for the first 4 joints!
    if (!torque_control_ || (req.id > 4) || (req.id < 1) ) 
    {
        res.result = false;
        return false;
    }

    SetJointTorque(req.id, req.goal_position);
    res.result = true;
    return true;

}


void GazeboInteface::goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr & msg) {
    if (msg->position.size() != 5) {
        ROS_INFO("Invalid joint state, execution aborted");
    } else {

        for (int index = 0; index < 5; index++) {

            std_msgs::Float64 msg_joint;
            msg_joint.data = msg->position.at(index);
            pub_arr[index].publish(msg_joint);
        }
    }
}

void GazeboInteface::recordArmJoints(const sensor_msgs::JointState::ConstPtr & msg) {
    int msgSize = msg->position.size();
    if (msgSize == 9) {
        for (int index = 0; index < 7; ++index)
            arm_state[index] = msg->position.at(index + 2); //the index of arm joints starts at 2
    } else if (msgSize == 2) {
        // wheel joint states
    } else {
        ROS_INFO("Invalid joint state, aborted");
    }
}

void GazeboInteface::stopExecution(const std_msgs::Empty & msg) {
    std_msgs::Float64 msg_joint;
    for (int index = 0; index < 5; index++) {
        msg_joint.data = arm_state[index];
        pub_arr[index].publish(msg_joint);
    }
}

void GazeboInteface::gripperStateCallback(const ros::TimerEvent &)
{
    std_msgs::Int8 gripper_state_msg;
    gripper_state_msg.data = gripper_state;
    gripper_state_pub.publish(gripper_state_msg);
}

void GazeboInteface::gripperOpen(const std_msgs::Empty & mg) {
    gripper_state = -1; // unknown
    std_msgs::Float64 msg;
    msg.data = -1*GRIPPER_OPEN_POS;
    joint_6_pub.publish(msg);
    msg.data = GRIPPER_OPEN_POS;
    joint_7_pub.publish(msg);

    ros::Duration(1).sleep();
    
    if (!isClose(arm_state[5], -1*GRIPPER_OPEN_POS) || 
        !isClose(arm_state[6], GRIPPER_OPEN_POS))
        gripper_state = -1;  // unknown
    else
        gripper_state = 0; // open
}

void GazeboInteface::gripperClose(const std_msgs::Empty & mg) {
    gripper_state = 1; // closing

    std_msgs::Float64 msg;
    msg.data = GRIPPER_CLOSE_POS;
    joint_6_pub.publish(msg);
    joint_7_pub.publish(msg);

    ros::Duration(1).sleep();

    if (!isClose(arm_state[5], GRIPPER_CLOSE_POS) || 
        !isClose(arm_state[6], GRIPPER_CLOSE_POS))
        gripper_state = 2;    //object in hand
    else
        gripper_state = 3; //Fully closed
}

void GazeboInteface::executeJointTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr & goal) {

    ros::Time trajectoryStartTime = ros::Time::now();
    ros::Rate loop_rate(25);

    if (goal->trajectory.points.size() < 2) {
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
        smooth_joint_trajectory_server_->setSucceeded(result);
        return;
    }
    for (int i = 0; i < goal->trajectory.points.size(); i++) {
        // boost::recursive_mutex::scoped_lock lock(api_mutex);
        //check for preempt requests from clients
        if (smooth_joint_trajectory_server_->isPreemptRequested()) {

            //preempt action server
            smooth_joint_trajectory_server_->setPreempted();
            ROS_INFO("Joint trajectory server preempted by client");
            return;
        }

        while ((ros::Time::now() - trajectoryStartTime).toSec() <
            goal->trajectory.points[i].time_from_start.toSec()) {
            continue;
        }

        // Publish on each individual joint topic
        for (int index = 0; index < NUMBER_OF_ARM_JOINTS; index++) {

            std_msgs::Float64 msg;
            msg.data = goal->trajectory.points[i].positions[index];
            pub_arr[index].publish(msg);
        }

        loop_rate.sleep();
    }

    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    smooth_joint_trajectory_server_->setSucceeded(result);
    ROS_INFO("Trajectory goal execution took %fs.",
        (ros::Time::now() - trajectoryStartTime).toSec());
}

int main(int argc, char ** argv) {
    // Init ROS node
    ros::init(argc, argv, "gazebo_locobot_control");
    GazeboInteface gzi;
    ros::spin();

    return 0;
}
