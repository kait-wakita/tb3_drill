#!/usr/bin/env python
import roslib
import rospy
import math
import tf, tf2_ros
import geometry_msgs.msg
import numpy as np
from sensor_msgs.msg import LaserScan

if __name__ == '__main__':
    rospy.init_node('tf_tb3')
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    rate = rospy.Rate(1)
    rospy.sleep(0.0) 

    while not rospy.is_shutdown():
        try:
            origin2base = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(10.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn( 'tf not found' )
        x = origin2base.transform.translation.x
        y = origin2base.transform.translation.y
        quaternion = origin2base.transform.rotation
        eul_d = np.rad2deg(tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w)))
        
        rospy.loginfo("x=%f, y=%f, theta=%f",x,y,eul_d[2])

        rate.sleep()
