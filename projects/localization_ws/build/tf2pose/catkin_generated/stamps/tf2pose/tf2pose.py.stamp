#!/usr/bin/env python
from __future__ import division
from __future__ import print_function

import sys
import time
import rospy
import tf

import numpy as np

import message_filters

from geometry_msgs.msg import PoseStamped


class TFtoPose:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.world_frame_id = 'map'
        self.robot_frame_id = 'base_link'
        self.pose_pub = rospy.Publisher("/conversion/pose", PoseStamped, queue_size = 1)
        publish_rate = 10
        self.period = 1 / publish_rate
        
        rospy.sleep(10)

    def publish_pose_from_tf(self):
        (translation, rotation) = self.tf_listener.lookupTransform(self.world_frame_id,self.robot_frame_id, rospy.Time(0))
        goal = PoseStamped()
        goal.header.frame_id = self.world_frame_id
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = translation[0]
        goal.pose.position.y = translation[1]
        goal.pose.position.z = translation[2]
        goal.pose.orientation.x = rotation[0]
        goal.pose.orientation.y = rotation[1]
        goal.pose.orientation.z = rotation[2]
        goal.pose.orientation.w = rotation[3]

        self.pose_pub.publish(goal)

def main(args):
    rospy.init_node('tf_to_pose', anonymous=True)
    tf_to_pose = TFtoPose()
    try:
        while True:
            tf_to_pose.publish_pose_from_tf()
            rospy.sleep(tf_to_pose.period)
    except KeyboardInterrupt:
        print("TF to pose conversion stopped!")


if __name__ == '__main__':
    main(sys.argv)
