#!/usr/bin/env python
import cv2
import numpy as np
import math
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading
from std_msgs.msg import Float64, ColorRGBA, Boolean
from geometry_msgs.msg import Point
from racecar.msg import BlobDetections

class EmergencyStop:
	def __init__(self):
		#Listeners

		self.lidar_detect = rospy.Subscriber("/racecar/laser/scan", LaserScan, self.LidarScanReceived) #Only used to call the emergency stop program if necessary.
						
		#Talkers

		#self.pub_stop_func = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=5)
		self.pub_stop_func = rospy.Publisher("/vesc/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=5) #Overrides any other command to the input/navigation topic
		#Intialize Messages
		self.drive_msg = AckermannDriveStamped

	

	def LidarScanReceived(self, msg):
		distances = msg.ranges #Gets a list that contains data about the distance of the nearest object for each degree measure
		if (min(scans[520:560]) < 1.5):	#If the mininimum of the distances to the nearest object for all angles measured between the array locations 520 and 560 is less than 1.5, stop moving.	
			self.drive_msg.drive.speed = 0
			self.pub_stop_func.publish(drive_msg)


if __name__=="__main__":
	rospy.init_node('EmergencyStop')
	e = EmergencyStop()
	rospy.loginfo("Intializing Emergency Stop")
	rospy.spin()

