/**
*   \file main.cpp
*   \brief Wrapper for the set_target_node.
*   \author Michał Krępa
*   \version 0.2
*   \date 10/04/2024
* 
*   Wrapper for set_target_node used in the second assignment of Research Track 1.
*
**/
#include <rt_assignment_2/set_target_node.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>

#include <assignment_2_2023/PlanningAction.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_target_node");
    auto nh = std::make_shared<ros::NodeHandle>();
    auto ac = std::make_shared<actionlib::SimpleActionClient<assignment_2_2023::PlanningAction>>("/reaching_goal", false);

    try
    {
        rt_assignment_2::SetTargetNode set_target_node(ac, nh);
        ROS_INFO("set_target_node ready");

        ros::spin();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    std::cout << "[" << ros::this_node::getName() << "]"
              << " Shutting down" << std::endl;
              
    ros::shutdown();
    return 0;
}