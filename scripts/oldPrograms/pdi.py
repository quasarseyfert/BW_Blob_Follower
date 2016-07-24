#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import time, collections

global prev_times, prev_errors

def pdi_callback(msg):
    kp=.5
    kd=.05

    #mult = 1  # right
    mult = -1 # left
    avgDist = sum(side) / len(side)
    error = ddes - avgDist

    if self.subdaniel == "g":
	side = msg.ranges[148:213]
    elif self.subdaniel = "r":
	side = msg.ranges[868:933]

    if abs(error) < .03:
        driving.drive.steering_angle = 0
    else:
        driving.drive.steering_angle =mult*pdi(kp, kd, error)
    pdi_pub.publish(driving)

def pdi(kp,kd,error):

    e_deriv = (error - prev_errors.popleft()) / (time.clock() - prev_times.popleft())
    global prev_times, prev_errors
    prev_times.append(time.clock())
    prev_errors.append(error)
    return kp*error + kd * e_deriv

rospy.init_node("pdi")
driving = AckermannDriveStamped()
driving.header.stamp=rospy.Time.now()
driving.drive.speed = 4
ddes = .5
#prev_error = 0
#time_prev = time.clock()
prev_times = collections.deque([time.clock() for _ in range(10)])
prev_errors = collections.deque([0 for _ in range(5)])

self.sublidar = rospy.Subscriber("/scan", LaserScan, self.laser_callback, queue_size=10)
self.subdaniel = rospy.Subscriber("/blob_color", ColorRGBA, self.pdi_callback)
self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size
rospy.spin()
