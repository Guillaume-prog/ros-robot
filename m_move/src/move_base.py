#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class MoveBase:

    def __init__(self):
        rospy.init_node('move_base')

        rospy.Subscriber('/cmd_vel', Twist, self.move)

    def move(msg):
        print(msg)
    
            

if __name__ == "__main__":
    MoveBase()
    while not rospy.is_shutdown:
        rospy.spin()
