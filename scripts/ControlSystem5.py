#!/usr/bin/env python

#This control system revision does not attemp to turn. The turning part will be done by the lidar detections instead. HTe lidar should have the car turn in the right way anyways.
import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped


from racecar.msg import BlobDetections
from std_msgs.msg import Float64
from std_msgs.msg import ColorRGBA
from std_msgs.msg import String
from geometry_msgs.msg import Point

import numpy as np


Stage_2 = False
class MoveTowardImage:
	def __init__(self):			#Don't put while lops in call backs. It will overload a message stream
		
		#Listeners. Listeners's third parameter are always callback functions. Listeners never have queue_sizes.

		self.blob_detect = rospy.Subscriber("/blob_detections", BlobDetections, self.MoveToBlob)	#blob_detect is an object that contains data about multiple blobs. The biggest blob is the one that has index 0. The same indices correspond to the same blob over the multiple data types in the message fields.


		#Talkers
		self.pub_mov_func = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=5)
		self.pub_blob_color_func = rospy.Publisher("/blob_color", String, queue_size=1)	#This is used to talk to the wall follow node. talkers setup the topics that subscribers/listeners subscribe/listen to

		#Fields/Constant. Most of the things here are not used yet (ie the things in the state and constants sections)
		# Messages
		self.drive_msg = AckermannDriveStamped #Sets the drive message to the  AckermannDriveStamped message type
		self.blob_color = ColorRGBA

		
		# State

		self.pid = {'p': 0.5, 'd': 1, 'i': 0}
		self.distance = {'desired': 0.5, 'last': 1, 'direction': 'right'}

		# Constants
		self.speed = 2
		self.collisionDistance = 1.5
		self.angles = {'left': 780, 'right': 300, 'sideRange': 10,
		               'frontRange': 15}
		


		
	def MoveToBlob(self, blob_detect): #Moves to the largest [choose a color] blob
		self.drive_msg.drive.speed = 2	#Sets a field of the drivemsg message.
				
		#Gets the position of the largest blob. The first will be the largest  blob because the blobs are sorted by size. (index 0 is biggest size). The next three lines
		#define a specific color that we want to find/move to
		
		if (len(blob_detect.colors) != 0):	#Make sure the blob detect arrays are not empty
			largest_blob = blob_detect.sizes[0]
			largest_blob_pos = blob_detect.locations[0]
			blob_x_pos = largest_blob_pos.x		#This will be in between 0 and 1 (inclusive) (0 is the left and 1 is the rihgt)
			blob_y_pos = largest_blob_pos.y		#Same thing here
			blob_color = blob_detect.colors[0]	#In rgba
			blob_area = blob_detect.size[0] #Get the area of the largest block of the set of the blocks with the color we specified
		else:
			rospy.loginfo("No blob!")
			return

		


		if (blob_area < 10000 and self.stage_2 == False):	#We are relatively far away form the blob and we are not in stage 2 (we need this second part because we could be far away from the block and be in stage_2 but in stage_2 we want to wallfollow
		#While we are relatively far away from the blob (ie blob size is small relative to screen), we drive towards the center of the blob. 
			if (blob_x_pos > 0.55): #If Blob is On the right side
				drive_msg.drive.steering_angle = .33 #Turn left
			elif (blob_x_pos < 0.45): #If Blob is on the left side.
				drive_msg.drive.steering_angle = -0.33 #Turn right
			else:	#If blob_x_pos (the x component of the blob's centroid) is in the middle 200 pixels, just move forward
				drive_msg.drive.steering_angle = 0
		elif (blob_area > 10000 and self.stage_2 == False):
			self.stage_2 = True
			

		if (self.stage_2 == True):
			#Intiate wall follow by publishing the color to the wall follow's topic.
			color = "r" if (blob_color.r == 255) else "g"
			self.pub_blob_color_func(color)
			
		#We don't have to worry about if the blob_area becomes too big relative to the screen (which would indicate we are close to the blob(?)) since the emergency stop node has priority over the navigation input topic and publishes speed = 0 if we are a certain distance from an object

if __name__=="__main__":
	rospy.init_node('ControlSystem')
	CC = MoveTowardImage()
	rospy.loginfo("Initializing Control System")
	rospy.spin()


	
		
		
	
		






