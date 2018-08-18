#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# Author: Michael Ferguson

import rospy
import actionlib
from fetch_auto_dock_msgs.msg import DockAction, DockGoal

if __name__ == "__main__":
    rospy.init_node("dock_script")
    ACTION_NAME = "/dock"
    rospy.loginfo("Connecting to %s..." % ACTION_NAME)
    client = actionlib.SimpleActionClient(ACTION_NAME, DockAction)
    client.wait_for_server()
    rospy.loginfo("Sending dock goal...")
    goal = DockGoal()
    client.send_goal(goal)
    rospy.loginfo("Done, press Ctrl-C to exit")
    rospy.spin()
