/*
 * Node that implements an action client, allowing the user to set a target to action server or to cancel it
 * Once the target has been reached there should be an option to enter a new target once the initial target has been reached
 * Node also should publish robot position and velocity as a custom message Pos(x y ) Vel(x Y z) and should be getting it from topic /odom
 * */

#include "setTarget.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/AveragingAction.h>

#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>

#include <rt_assignment_2/setTargetAction.h>
#include <rt_assignment_2/robotState.h>
#include <rt_assignment_2/target.h>

#include <assignment_2_2023/PlanningAction.h>

ros::Publisher pub_state;
ros::Publisher pub_target;



void robotTargetCallback(rt_assignment_2::target::ConstPtr& msg)
{
  char type;
  float x;
  float y;

  int result = std::sscanf(target_str.c_str(), "%c %f %f", &type, &x, &y);
}

planningGoal createPlanningGoal(float x, float y)
{
  planningGoal goal;
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  return goal;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "setTarget");

  /* create the action client */
  Client ac("/reaching_goal"/*, true*/);
  // boost::thread spin_thread(&spinThread);

  ros::NodeHandle nh;
  // Publisher to robot's Current state message
  pub_state = nh.advertise<rt_assignment_2::robotState>("/robotState", 5);
  pub_target = nh.advertise<rt_assignment_2::target>("/robot_target", 5);

  ROS_INFO("Client sent goal: x = %d y = %d to Server", des_x, des_y);

  ros::Subscriber sub_state = nh.subscribe("/odom", 10, robotStateCallback);
  ros::Subscriber sub_state = nh.subscribe("/robot_target", 10, robot_targetCallback);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  while (ros::ok())
  {
    ROS_INFO("Waiting for new target. ");
    std::string target_str;

    if (!nh.hasParam("/robot_target"))
    {
      ROS_INFO("No target provided.");
      ros::Duration(50.0);
      continue;
    }
    else
    {
      ROS_INFO("Target received!");
      ros::param::get("/robot_target", target_str);
      // Check the correctness of target param
      if (target_str[0] == 'c')
      {
        // cancel goal
      }
      else if (target_str[0] == 's')
      {
        // set the goal, parse target_str
        //  parsing target_str

        
        ac.sendGoal(goal);
        ROS_INFO("Client sent new goal = VALUES ");
        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
        if (finished_before_timeout)
        {
          actionlib::SimpleClientGoalState state = ac.getState();
          ROS_INFO("Action finished: %s", state.toString().c_str());
        }
        else
        {
          ROS_INFO("Action did not finish before the time out.");
        }
      }
    }
  }
  

  ros::spin();
  // spin_thread.join();

  // return 0;
}



void robotStateCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  /* Receive odometry message */
  int32_t x_pos = msg->pose.pose.position.x;
  int32_t y_pos = msg->pose.pose.position.y;
  int32_t x_vel = msg->twist.twist.linear.x;
  int32_t y_vel = msg->twist.twist.linear.y;
  ROS_INFO("Received Odometry from '/odom' x = %d, y = %d, vel_x = %d, vel_y = %d",
           x_pos, y_pos, x_vel, y_vel);

  /* Publish robotState messsage */
  rt_assignment_2::robotState robotState_msg;
  robotState_msg.x_pos = x_pos;
  robotState_msg.y_pos = y_pos;
  robotState_msg.x_vel = x_vel;
  robotState_msg.y_vel = x_vel;
  pub_state.publish(robotState_msg);
  ROS_INFO("RobotState message successfully published");
}

void spinThread()
{
  ros::spin();
}