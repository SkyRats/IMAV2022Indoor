#!/usr/bin/env python3
import rospy
import tf
import numpy as np

if __name__ == '__main__':
    rospy.init_node('odom_tf_listener')
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            position,quaternion = listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
            print(np.multiply(-1,position))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("position getter failed")
            continue