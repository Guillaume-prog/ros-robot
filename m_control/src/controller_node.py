#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from joystick_obj import *


class ControllerNode(Controller):

    def __init__(self):
        super().__init__()
        rospy.init_node('joystick_controller')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.send_command, oneshot=False)
        rospy.on_shutdown(self.shutdown)
        
        self.x = 0
        self.y = 0

    def shutdown(self):
        self.running = False


    def send_command(self, msg):
        pub_msg = Twist()
        self.pub.publish(pub_msg)


    def on_button_pressed(self, button_id):
        if button_id == BUTTON_TRIANGLE:
            print('hi')

    def on_joystick_move(self, joystick_id, value):
        if joystick_id == L3_X:
            self.x = value
        elif joystick_id == L3_Y:
            self.y = value

        print(self.x, self.y)

    


if __name__ == "__main__":
    ControllerNode()
    while not rospy.is_shutdown:
        rospy.spin()
    