#!/usr/bin/env python3
""" 
This script broadcasts a tf frame called odom_rot that spins about the z axis
at a slow rate but also allows its yaw to be set at the command line.
The yaw is listed on the command line in radians.
"""
# Copyright (c) 2016, Fetch Robotics Inc.
# Author: Griswald Brooks

import rospy
import tf
import threading

class OdomTransform(object):
    def __init__(self):
        self.x = 1
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

def updateTransform(odom_transform):
    while not rospy.is_shutdown():
        new_yaw = input("Enter new yaw[" + str(odom_transform.yaw) + "]: ")
        
        try:
            odom_transform.yaw = float(new_yaw)
        except ValueError:
            pass

def main():

    # Create a node.
    rospy.init_node("odom_rot")

    # Create transform object
    odom_rot = OdomTransform()

    # Launch keyboard thread.
    kb_th = threading.Thread(target=updateTransform, args=(odom_rot,))
    kb_th.daemon = True
    kb_th.start()

    # Setup broadcast rate.
    r = rospy.Rate(100)

    # Broadcast odom_rot
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        br.sendTransform((odom_rot.x, odom_rot.y, odom_rot.z),
                         tf.transformations.quaternion_from_euler(odom_rot.roll, odom_rot.pitch, odom_rot.yaw),
                         rospy.Time.now(),
                         "odom_rot",
                         "odom")
        # Increment yaw
        odom_rot.yaw += 0.0001

        r.sleep()

if __name__ == "__main__":

    main()
