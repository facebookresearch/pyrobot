/**
 *  @file   turtlebot_controller_node.h
 *  @brief  Turtlebot SE(2) non-holonomic base controller
 *  @author Mustafa Mukadam
 *  @date   Jan 26, 2018
 **/

#ifndef TURTLEBOT_CONTROLLER_NODE_H_
#define TURTLEBOT_CONTROLLER_NODE_H_

#include <iostream>
#include <memory>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>


namespace turtlebot {

class TurtlebotController
{
  public:
    typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> TrajectoryServer;

  private:
    ros::Publisher base_vel_pub_;
    std::unique_ptr<TrajectoryServer>  base_trajectory_server_;

  public:
    /// Default constructor
    TurtlebotController() {}

    /**
     *  Turtlebot controller
     *
     *  @param nh node handle
     **/
    TurtlebotController(ros::NodeHandle nh);

    /// Default destructor
    virtual ~TurtlebotController() {}

    /**
     *  SE(2) non-holonomic base open-loop velocity controller
     *
     *  @param goal action goal
     **/
    void executeBaseTajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
};

} // namespace turtlebot

#endif
