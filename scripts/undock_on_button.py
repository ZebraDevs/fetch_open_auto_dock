#!/usr/bin/env python3

# Copyright (c) 2015, Fetch Robotics Inc.
# Author: Michael Ferguson

import rospy
import actionlib
from sensor_msgs.msg import Joy
from fetch_auto_dock_msgs.msg import UndockAction, UndockGoal

# Listen to joy messages, when button held, undock
class UndockTeleop:
    ACTION_NAME = "/undock"

    def __init__(self):
        rospy.loginfo("Connecting to %s..." % self.ACTION_NAME)
        self.client = actionlib.SimpleActionClient(self.ACTION_NAME, UndockAction)
        self.client.wait_for_server()
        rospy.loginfo("Done.")

        self.dock_button = rospy.get_param("~undock_button", 15)  # default button is the square

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
            rospy.logwarn("undock_button is out of range")

    def reset(self):
        goal = UndockGoal()
        goal.rotate_in_place = True
        self.client.send_goal(goal)

if __name__ == "__main__":
    rospy.init_node("undock_on_button")
    c = UndockTeleop()
    rospy.spin()
