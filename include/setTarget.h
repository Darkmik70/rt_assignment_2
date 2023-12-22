#ifndef RT_ASSIGNMENT_2_ACLIENT_H
#define RT_ASSIGNMENT_2_ACLIENT_H


#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <rt_assignment_2/setTargetAction.h>
#include <assignment_2_2023/PlanningAction.h>

// typedef actionlib::SimpleActionClient<assignment_2_2023::PlanningAction> Client;
typedef assignment_2_2023::PlanningGoal planningGoal;

planningGoal createPlanningGoal(float x, float y);

void robotStateCallback(const nav_msgs::Odometry::ConstPtr& msg);


void spinThread();

#endif // RT_ASSIGNMENT_2_ACLIENT_H