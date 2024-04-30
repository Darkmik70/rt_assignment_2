#ifndef RT_ASSIGNMENT_2_SET_TARGET_HPP
#define RT_ASSIGNMENT_2_SET_TARGET_HPP


#include <ros/ros.h>

#include <rt_assignment_2/robotTarget.h>
#include <rt_assignment_2/robotCancelGoal.h>
#include <rt_assignment_2/robotState.h>

#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>

#include <assignment_2_2023/PlanningAction.h>



namespace rt_assignment_2
{
    class SetTargetNode
    {
    public:
        SetTargetNode(
            const std::shared_ptr<actionlib::SimpleActionClient<assignment_2_2023::PlanningAction>> &ac,
            const std::shared_ptr<ros::NodeHandle> &nh);
        ~SetTargetNode();

    private:
        void actualPoseCallback(const ros::TimerEvent &event);
        void cancelGoalCallback(const rt_assignment_2::robotCancelGoal::ConstPtr &msg);
        void setNewTargetCallback(const rt_assignment_2::robotTarget::ConstPtr &msg);
        void robotStateCallback(const nav_msgs::Odometry::ConstPtr &msg);

        void doneCallback(const actionlib::SimpleClientGoalState &state, const assignment_2_2023::PlanningResultConstPtr &result);
        void activeCallback();
        void feedbackCallback(const assignment_2_2023::PlanningFeedbackConstPtr &feedback);


        std::shared_ptr<actionlib::SimpleActionClient<assignment_2_2023::PlanningAction>> ac_;
        std::shared_ptr<ros::NodeHandle> nh_;


        ros::Subscriber sub_state_;
        ros::Subscriber sub_target_;
        ros::Subscriber sub_cancel_;

        ros::Publisher pub_state_;

        ros::Timer get_actual_pose_timer_;

        double feedback_pos_x_;
        double feedback_pos_y_;
    };
}

#endif // RT_ASSIGNMENT_2_SET_TARGET_HPP
