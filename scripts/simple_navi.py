#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def callback_laser(msg):
    global distance
    data = np.array(msg.ranges)
    data = np.where(data<msg.range_min, msg.range_max, data)
    distance = min(min(data[0:30]), min(data[330:360]))

def callback_shutdown():
    cmd_vel_pub.publish(stop_cmd)

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
scan_sub = rospy.Subscriber('scan', LaserScan, callback_laser)

rospy.init_node('simple_navi') 
listener = tf.TransformListener()

move_cmd = Twist()
move_cmd.linear.x = 0.2
rotate_cmd = Twist()
rotate_cmd.angular.z = 0.2
stop_cmd = Twist()

distance = 100
x_goal = 1.8
y_goal = 0.0

rate = rospy.Rate(2)
rospy.on_shutdown(callback_shutdown)
rospy.sleep(0.0)  # for gazebo

while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    e = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))
    # rospy.loginfo("x=%f, y=%f, theta=%f",trans[0],trans[1],np.rad2deg(e[2]))

    x = trans[0]
    y = trans[1]
    r = math.atan2(y_goal-y, x_goal-x)
    rtd = np.rad2deg(r - e[2])
    dir_goal = (rtd + 180) % 360 - 180
    dist_goal=math.sqrt((x_goal-x)**2+(y_goal-y)**2)

    rospy.loginfo("dir(pose)=%f, dir(goal)=%f", np.rad2deg(e[2]), np.rad2deg(r))  

    if dist_goal < 0.3:
        cmd_vel_pub.publish(stop_cmd)
    if distance < 0.5:
        cmd_vel_pub.publish(rotate_cmd)
        a = 0
    else:
        navi_cmd = geometry_msgs.msg.Twist()
        navi_cmd.linear.x = min(dist_goal / 1, 0.3)
        navi_cmd.angular.z = 0.5 * (np.deg2rad(dir_goal))
        cmd_vel_pub.publish(navi_cmd)

    rate.sleep()
