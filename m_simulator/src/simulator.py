#!/usr/bin/env python3

import pygame
from pygame import display, draw, Rect

import threading
import rospy

from m_control.msg import MoveCommand


class Simulator:

    def __init__(self):
        rospy.init_node('move_base')
        rospy.on_shutdown(self.shutdown)

        self.left_motor = 0
        self.right_motor = 0
        self.command_msg = MoveCommand()
        self.data_sub = rospy.Subscriber('/cmd_vel', MoveCommand, self.command_cb)

        pygame.init()
        display.set_caption("ROS Simulator")
        self.screen = display.set_mode((800, 600))

        # main pygame thread
        self.running = True
        self.game_thread = threading.Thread(target=self.main_loop)
        self.game_thread.start()


    def main_loop(self):
        clock = pygame.time.Clock()
        while self.running:
            clock.tick(60)

            self.render_ui()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False


    def render_ui(self):
        self.screen.fill((255,255,255))
        
        self.draw_bar(0, self.command_msg.linear)
        self.draw_bar(1, self.command_msg.angular)
        self.draw_bar(2, self.command_msg.speed)

        self.draw_bar(3, self.left_motor, color=(0,0,255))
        self.draw_bar(4, self.right_motor, color=(0,0,255))

        display.flip()

    def draw_bar(self, index, value, color=(255,0,0)):
        height = value * 100

        x_offset = 185 + index * 100
        y_offset = 300 - (height if height > 0 else 0)
        
        draw.rect(self.screen, (200,200,200), Rect(x_offset, 200, 30, 200))
        draw.rect(self.screen, color, Rect(x_offset, y_offset, 30, abs(height)))

    def shutdown(self):
        self.running = False

    def command_cb(self, msg):
        self.command_msg = msg

        if(self.is_clamped(msg.linear, 0.1)):
            self.left_motor = msg.angular
            self.right_motor = -msg.angular
        else:
            self.left_motor = msg.linear + (msg.angular/2 if msg.angular < 0 else 0)
            self.right_motor = msg.linear - (msg.angular/2 if msg.angular > 0 else 0)
            pass

    def is_clamped(self, value, bounds):
        return value > -bounds and value < bounds

if __name__ == "__main__":
    Simulator()
    while not rospy.is_shutdown:
        rospy.spin()