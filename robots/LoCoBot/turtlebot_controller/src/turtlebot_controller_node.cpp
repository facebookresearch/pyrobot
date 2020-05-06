/**
 *  @file   turtlebot_controller_node.h
 *  @brief  Turtlebot SE(2) non-holonomic base controller
 *  @author Mustafa Mukadam
 *  @date   Jan 26, 2018
 **/

#include <turtlebot_controller/turtlebot_controller_node.h>
#include <algorithm>    // std::min
namespace turtlebot {

/* ************************************************************************** */
TurtlebotController::TurtlebotController(ros::NodeHandle nh)
{
  base_trajectory_server_ = std::unique_ptr<TrajectoryServer>(new
    TrajectoryServer(nh, "/turtle/base_controller/trajectory",
    boost::bind(&TurtlebotController::executeBaseTajectory, this, _1), false));
  base_trajectory_server_->start();
  base_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
}

/* ************************************************************************** */
void TurtlebotController::executeBaseTajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
  // check if valid trajectory
  if (goal->trajectory.points.size() < 2)
  {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    base_trajectory_server_->setSucceeded(result);
    return;
  }

  //ROS_INFO("Starting turtlebot base trajectory");
  double sec = (goal->trajectory.points[1].time_from_start - goal->trajectory.points[0].time_from_start).toSec();
  geometry_msgs::Twist cmd_vel;
  ros::Rate rate(1/sec);

  double for_start_time;
  double cur_time;
  double acc_x;
  double acc_theta;

  for (int i=0; i<goal->trajectory.points.size()-1; i++)
  {

    for_start_time = ros::Time::now().toSec();

    if (base_trajectory_server_->isPreemptRequested()) {

      //preempt action server
      base_trajectory_server_->setPreempted();
      ROS_INFO("Turtlebot trajectory server preempted by client");
      /*cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      base_vel_pub_.publish(cmd_vel);*/
      return;
    }

    acc_x = (goal->trajectory.points[i+1].velocities[0] - goal->trajectory.points[i].velocities[0])/sec;
    acc_theta = (goal->trajectory.points[i+1].velocities[2] - goal->trajectory.points[i].velocities[2])/sec;
    
    //cmd_vel.linear.x = goal->trajectory.points[i].velocities[0];
    //cmd_vel.angular.z = goal->trajectory.points[i].velocities[2];
    cmd_vel.linear.y = 0.0;//goal->trajectory.points[i].velocities[1];

    cur_time = ros::Time::now().toSec();

    while (cur_time - for_start_time < sec)
    {

      ROS_INFO("Turtlebot tr");
      base_vel_pub_.publish(cmd_vel);
      cmd_vel.linear.x  = goal->trajectory.points[i].velocities[0] + acc_x *(cur_time - for_start_time);
      cmd_vel.angular.z = goal->trajectory.points[i].velocities[2] + acc_theta *(cur_time - for_start_time);
      base_vel_pub_.publish(cmd_vel);
      cur_time = ros::Time::now().toSec();


      if (base_trajectory_server_->isPreemptRequested()) 
      {
        //preempt action server
        base_trajectory_server_->setPreempted();
        ROS_INFO("Turtlebot trajectory server preempted by client");
      /*  cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        base_vel_pub_.publish(cmd_vel);*/
        return;
      }

    } 
  }

  control_msgs::FollowJointTrajectoryResult result;
  result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
  base_trajectory_server_->setSucceeded(result);
  //ROS_INFO("Turtlebot base trajectory complete.");
}

} // namespace turtlebot

/* ************************************************************************** */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_controller_node");
  ros::NodeHandle nh;

  turtlebot::TurtlebotController robot(nh);
  ros::spin();
}
