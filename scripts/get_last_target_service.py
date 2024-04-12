"""
ROSservice Server for obtaining the most recent target sent by the user.

This module provides a ROS service server for obtaining the most recent target sent by the user to the robot.

Author: Michal Krepa
Version: 0.1
Date: 10/04/2024

Subscribes to:
- `/robot_target`

Publishes to:
- [None]

Service:
- `get_last_target`
"""

import rospy

from rt_assignment_2.srv import LastTarget, LastTargetResponse
from rt_assignment_2.msg import RobotTarget

class LastTargetServer:
    """
    ROS service server for handling the latest target sent by user to the robot.
    """
    def __init__(self, name: str) -> None:
        """
        Constructor for LastTargetServer class.

        Initializes a ROS node and sets up subscribers and services.

        Parameters:
            name (str): The name of the ROS node.
        """
        rospy.init_node(name)
        self.last_target = None
        rospy.Subscriber("/robot_target", RobotTarget, self.robot_target_callback)
        self.service = rospy.Service('get_last_target', LastTarget, self.handle_get_last_target)
        rospy.loginfo("Service ready to provide the last target coordinates.")
    
    def robot_target_callback(self, data):
        """
        Callback function for handling the latest robot's target position sent by the user.

        Parameters:
            data (RobotTarget): The latest robot's target position.
        """
        self.last_target = data

    def handle_get_last_target(self, req):
        """
        Handles the 'get_last_target' service.

        Parameters:
            req: The service request.

        Returns:
            LastTargetResponse: The service response containing the last target coordinates.
        """
        response = LastTargetResponse()
        if self.last_target:
            response.x = self.last_target.x
            response.y = self.last_target.y
        else:
            rospy.logwarn("No target received yet.")
        return response
    
def main():
    """
    Main function to start the ROS node and spin the LastTargetServer.
    """
    last_target_server_node = LastTargetServer('get_last_target')
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
