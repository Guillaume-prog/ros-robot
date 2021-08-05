#!/usr/bin/env python3
import rospy
from m_control.msg import MoveCommand
from joystick_obj import *


class ControllerNode(Controller):

    def __init__(self):
        super().__init__()
        rospy.init_node('joystick_controller')

        self.pub_msg = MoveCommand()

        self.pub = rospy.Publisher('/cmd_vel', MoveCommand, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.send_command, oneshot=False)
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.running = False

    def send_command(self, msg):
        self.pub.publish(self.pub_msg)

    def on_joystick_move(self, joystick_id, value):
        trunc_value = round(value if abs(value) > 0.1 else 0, 2)
        
        if joystick_id == L3_X:
            self.pub_msg.angular = trunc_value
        elif joystick_id == L3_Y:
            self.pub_msg.linear = -trunc_value
        elif joystick_id == R2_Y:
            self.pub_msg.speed = (trunc_value + 1) / 2

    


if __name__ == "__main__":
    ControllerNode()
    while not rospy.is_shutdown:
        rospy.spin()
    