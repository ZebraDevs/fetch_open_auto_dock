#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# Author: Michael Ferguson

import copy
from math import cos, sin

import rospy
import tf
from fetch_auto_dock_msgs.msg import DockActionFeedback
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class ViewDock(object):

    def __init__(self, publish_tf=True):
        rospy.init_node("view_dock")
        rospy.Subscriber("dock/feedback", DockActionFeedback, self.callback)
        self.publisher = rospy.Publisher("path", Path)
        self.broadcaster = tf.TransformBroadcaster()
        self.publish_tf = publish_tf

    def callback(self, msg):
        # Publish TF for dock
        if self.publish_tf:
            pose = msg.feedback.dock_pose.pose
            self.broadcaster.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                                           (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                                           msg.feedback.dock_pose.header.stamp,
                                           "dock",
                                           msg.feedback.dock_pose.header.frame_id)
        # Publish forward simulation of where current command will take robot
        path = Path()
        path.header.stamp = msg.feedback.dock_pose.header.stamp
        path.header.frame_id = "base_link"
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.orientation.w = 1.0
        path.poses.append(copy.deepcopy(pose))
        yaw = 0.0
        for i in range(20):
            pose.pose.position.x += 0.1 * msg.feedback.command.linear.x * cos(yaw)
            pose.pose.position.y += 0.1 * msg.feedback.command.linear.x * sin(yaw)
            yaw += 0.1 * msg.feedback.command.angular.z
            pose.pose.orientation.z = sin(yaw/2.0);
            pose.pose.orientation.w = cos(yaw/2.0);
            path.poses.append(copy.deepcopy(pose))
        self.publisher.publish(path)

if __name__ == "__main__":
    v = ViewDock()
    rospy.spin()
