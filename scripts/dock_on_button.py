#!/usr/bin/env python3

# Copyright (c) 2015, Fetch Robotics Inc.
# Author: Michael Ferguson

import rospy
import actionlib
from sensor_msgs.msg import Joy
from fetch_auto_dock_msgs.msg import DockAction, DockGoal

# Listen to joy messages, when button held, dock
class DockTeleop:
    ACTION_NAME = "/dock"

    def __init__(self):
        rospy.loginfo("Connecting to %s..." % self.ACTION_NAME)
        self.client = actionlib.SimpleActionClient(self.ACTION_NAME, DockAction)
        self.client.wait_for_server()
        rospy.loginfo("Done.")

        self.dock_button = rospy.get_param("~dock_button", 13)  # default button is the circle

        self.pressed = False
        self.pressed_last = None

        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)

    def joy_callback(self, msg):
        try:
            if msg.buttons[self.dock_button] > 0:
                if not self.pressed:
                    self.pressed_last = rospy.Time.now()
                    self.pressed = True
                elif self.pressed_last and rospy.Time.now() > self.pressed_last + rospy.Duration(1.0):
                    self.reset()
                    self.pressed_last = None
            else:
                self.pressed = False
        except IndexError:
            rospy.logwarn("dock_button is out of range")

    def reset(self):
        goal = DockGoal()
        self.client.send_goal(goal)

if __name__ == "__main__":
    rospy.init_node("dock_on_button")
    c = DockTeleop()
    rospy.spin()
