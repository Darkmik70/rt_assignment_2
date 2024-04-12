"""
ROS service server for handling distance and average speed calculations based on the robot's target.

This module provides a ROS service server for handling distance and average speed calculations based on the robot's target.

Author: Michal Krepa
Version: 0.1
Date: 10/04/2024

Subscribes to:
- `/robot_target`
- `/robot_state`

Publishes to:
- [None]

Service:
- `get_target_distance`
"""

import rospy
from math import hypot

from rt_assignment_2.msg import RobotState
from rt_assignment_2.msg import RobotTarget
from rt_assignment_2.srv import TargetDistance, TargetDistanceResponse

class DistanceToTargetServer:
    """
    ROS service server for handling distance and average speed calculations based on the robot's target.
    """
    def __init__(self, name: str) -> None:
        """
        Constructor for DistanceToTargetServer class.

        Initializes a ROS node, sets up subscribers, and services.

        Parameters:
            name (str): The name of the ROS node.
        """
        rospy.init_node(name, anonymous=False)  
        
        self.robot_speed_list = []                                                     
        self.av_speed_window_limit = rospy.get_param('av_window_size', default=10)      
        self.robot_current = RobotState()                                               
        self.target_position = RobotTarget()                                            
        self.is_target_set = False      

        rospy.Subscriber("/robot_target", RobotTarget, self.robot_target_callback)     
        rospy.Subscriber("/robot_state", RobotState, self.robot_state_callback)
        self.service = rospy.Service("get_target_distance", TargetDistance, \
                                      self.handle_get_target_distance)
                                      

    def robot_target_callback(self, data):
        """
        Callback function for handling updates to the robot's target position.

        Parameters:
            data (RobotTarget): The latest robot's target position.
        """
        self.target_position.x = data.x
        self.target_position.y = data.y
        if not self.is_target_set:
            self.is_target_set = True

    def robot_state_callback(self, data):
        """
        Callback function for handling updates to the robot's current state.

        Parameters:
            data (RobotState): The current state of the robot.
        """
        self.robot_current.x_pos = data.x_pos
        self.robot_current.y_pos = data.y_pos
        if len(self.robot_speed_list) < self.av_speed_window_limit:
            self.robot_speed_list.append((data.x_vel, data.y_vel))
        elif len(self.robot_speed_list) == self.av_speed_window_limit:
            self.robot_speed_list.pop(0)
            self.robot_speed_list.append((data.x_vel, data.y_vel))

    def handle_get_target_distance(self, req):
        """
        Handles the 'get_target_distance' service.

        Parameters:
            req: The service request.

        Returns:
            TargetDistanceResponse: The service response containing calculated distance and average speed.
        """
        response = TargetDistanceResponse()
        if self.is_target_set:
            response.dist_x = self.target_position.x - self.robot_current.x_pos
            response.dist_y = self.target_position.y - self.robot_current.y_pos
            response.dist = hypot((self.target_position.x - self.robot_current.x_pos), \
                                  (self.target_position.y - self.robot_current.y_pos))
        else:
            response.dist_x = 0.0
            response.dist_y = 0.0
        response.av_speed_x = sum( x[0] for x in self.robot_speed_list) / len(self.robot_speed_list)
        response.av_speed_y = sum( y[1] for y in self.robot_speed_list) / len(self.robot_speed_list)
        return response

def main():
    """
    Main function to start the ROS node and spin the DistanceToTargetServer.
    """
    last_target_server_node = DistanceToTargetServer('get_target_distance')
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
