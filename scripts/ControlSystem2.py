#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped


from racecar.msg import BlobDetections
from std_msgs.msg import Float64
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

import numpy as np

class MoveTowardImage:
	def __init__(self):
		
		#Listeners. Listeners's third parameter are always callback functions. Listeners never have queue_sizes.

		self.blob_detect = rospy.Subscriber("/blob_detections", BlobDetections, self.MoveToBlob)	#blob_detect is an object that contains data about multiple blobs. The biggest blob is the one that has index 0. The same indices correspond to the same blob over the multiple data types in the message fields.
		self.lidar_detect - rospy.Subscriber("/racecar/laser/scan", LaserScan, self.LidarScanReceived) #Only used for emergency stop in this program.
		#Talkers
		self.pub_mov_func = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=5)
		
		#Fields/Constant. Most of the things here are not used yet (ie the things in the state and constants sections)
		# Messages
		self.drive_msg = AckermannDriveStamped #Sets the drive messaged to the AckermannDriveStamped message type

		# State
		self.pid = {'p': 0.5, 'd': 1, 'i': 0}
		self.distance = {'desired': 0.5, 'last': 1, 'direction': 'right'}

		# Constants
		self.speed = 2
		self.collisionDistance = 1.5
		self.angles = {'left': 780, 'right': 300, 'sideRange': 10,
		               'frontRange': 15}

	def LidarScanReceived(self, msg):
		distances = msg.ranges #Gets a list that contains data about the distance of the nearest object for each degree measure
		if (min(scans[520:560]) < 1.5):	#If the mininimum of the distances to the nearest object for all angles measured between the array locations 520 and 560 is less than 1.5, stop moving.	
			self.drive_msg.drive.speed = 0

		self.pub_mov_func.publish(self.drive_msg)
		
	def MoveToBlob(self): #Moves to the largest [choose a color] blob
		self.drive_msg.drive.speed = 2	#Sets a field of the drivemsg message.
				
		#Gets the position of the largest green blob. The first green blob will be the largest green blob because the blobs are sorted by size. (index 0 is biggest size). The next three lines
		#define a specific color that we want to find/move to
		ele = ColorRGBA()
		ele.r = 0
		ele.g = 255
		largest_green_blob_index = self.blob_detect.colors.index(ele)
		blob_pos = self.blob_detect.locations[largest_green_blob_index]  
					
		blob_x_pos = blob_pos[0]		#This will be in between 0 and 1 (inclusive) (0 is the left and 1 is the right (?))
		blob_y_pos = blob_pos[1]		#Same thing here
		
		blob_area = blob_detect.size[largest_green_blob_index] #Get the area of the largest block of the set of the blocks with the color we specified
		

		if (blob_area < 1000):	#Keep moving unless the blob_area becomes too big (meaning that we must be very close to the blob, ie it is taking up more of our field of vision.
			#If the blob_area is "small, we want to keep moving toward the blob while continually centering our field of vision with the blob.
			if (blob_x_pos > 0.55): #If Blob is On the right side
				drive_msg.drive.steering_angle = .33 #Turn left
			elif (blob_x_pos < 0.45): #If Blob is on the left side.
				drive_msg.drive.steering_angle = -0.33 #Turn right
			else:	#If blob_x_pos (the x component of the blob's centroid) is in the middle 200 pixels, just move forward
				drive_msg.drive.steering_angle = 0
			self.pub_mov_func.publish(drive_msg)
		else:	# if the contour is really big, we are close to the sign, so we do an emergency stop. This is a bit redundant with our emergency stop function that comes from the lidar scan.
			self.drive_msg.drive.speed = 0
			self.pub_mov_func.publish(drive_msg)

		


if __name__=="__main__":
	rospy.init_node('ControlSystem')
	rospy.loginfo("Initializing")
	rospy.spin()


	
		
		
	
		






