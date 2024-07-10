#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import numpy as np
from sensor_msgs.msg import LaserScan

if __name__ == '__main__':
    rospy.init_node('tf_tb3')
    listener = tf.TransformListener()
    
    rate = rospy.Rate(1)
    rospy.sleep(0.0) 

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        e = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))
        rospy.loginfo("x=%f, y=%f, theta=%f",trans[0],trans[1],e[2])

        rate.sleep()
