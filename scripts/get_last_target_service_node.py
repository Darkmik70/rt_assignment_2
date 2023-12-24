#!/usr/bin/env python

from rt_assignment_2.srv import GetLastTarget, GetLastTargetResponse
from rt_assignment_2.msg import robotTarget
import rospy

class LastTargetServer:
    def __init__(self):
        self.last_target = None
        rospy.Subscriber("/robot_target", robotTarget, self.robot_target_callback)
        self.service = rospy.Service('get_last_target', GetLastTarget, self.handle_get_last_target)
        rospy.loginfo("Service ready to provide the last target coordinates.")
    
    def robot_target_callback(self, data):
        self.last_target = data

    def handle_get_last_target(self, req):
        response = GetLastTargetResponse()
        if self.last_target is not None:
            response.x = self.last_target.x
            response.y = self.last_target.y
        else:
            rospy.logwarn("No target received yet.")
        return response

if __name__ == "__main__":
    rospy.init_node('get_last_target_server')
    last_target_server_node = LastTargetServer()
    rospy.spin()