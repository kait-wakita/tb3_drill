#!/usr/bin/env python
import rospy
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

rospy.init_node('auto') 

move_cmd = Twist()
move_cmd.linear.x = 0.2
rotate_cmd = Twist()
rotate_cmd.angular.z = 0.2
stop_cmd = Twist()

distance = 100

rate = rospy.Rate(10)
rospy.on_shutdown(callback_shutdown)
rospy.sleep(0.0) # for gazebo

while not rospy.is_shutdown():
    if distance < 0.5:
        cmd_vel_pub.publish(rotate_cmd)
    else:
        cmd_vel_pub.publish(move_cmd)

    rate.sleep()
