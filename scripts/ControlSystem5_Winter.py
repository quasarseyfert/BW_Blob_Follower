#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped


from racecar.msg import BlobDetections
from std_msgs.msg import Float64,ColorRGBA,String
from geometry_msgs.msg import Point

import numpy as np

global Stage_2, Turning
Wall_Follow = False
Turning = False
class MoveTowardImage:
	def __init__(self):			#Don't put while lops in call backs. It will overload a message stream
		
		#Listeners. Listeners's third parameter are always callback functions. Listeners never have queue_sizes.

		self.blob_detect = rospy.Subscriber("/blob_detections", BlobDetections, self.MoveToBlob)	#blob_detect is an object that contains data about multiple blobs. The biggest blob is the one that has index 0. The same indices correspond to the same blob over the multiple data types in the message fields.


		#Talkers
		self.pub_mov_func = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=5)
		self.pub_blob_color_func = rospy.Publisher("/blob_color", String, queue_size=1)	#This is used to talk to the wall follow node. talkers setup the topics that subscribers/listeners subscribe/listen to

		#Fields/Constant. Most of the things here are not used yet (ie the things in the state and constants sections)
		# Messages

		# Constants
		self.speed = 2


		
	def MoveToBlob(self, blob_detect): #Moves to the largest [choose a color] blob
                drive_msg = AckermannDriveStamped() #Sets the drive message to the  AckermannDriveStamped message type
		blob_Color = ColorRGBA()
		
		drive_msg.drive.speed = self.speed	#Sets a field of the drivemsg message.
				
		#Gets the position of the largest blob. The first will be the largest  blob because the blobs are sorted by size. (index 0 is biggest size).
		
		if (len(blob_detect.colors) == 0):	#Make sure the blob detect arrays are not empty
                        self.pub_mov_func.publish(drive_msg)
                        return#exit

                #if there are blobs
                largest_blob = blob_detect.sizes[0]
		largest_blob_pos = blob_detect.locations[0]
		blob_x_pos = largest_blob_pos.x		#This will be in between 0 and 1 (inclusive) (0 is the left and 1 is the right)
		blob_y_pos = largest_blob_pos.y		#Same thing here
		blob_color = blob_detect.colors[0]	#In rgba

                if (Wall_Follow == True and Turning == False):
			#Intiate wall follow by publishing the color to the wall follow's topic.
			color = "r" if (blob_color.r == 255) else "g"
			self.pub_blob_color_func(color)

			#stop further subscriptions and basically end the node
			self.blob_detect.unregister()
                else:
                        if (blob_area > 10000):	#We are close to the blob, so we start turning
                                #PUT TURN HERE
				Turning = True
				turning_start_time = rospy.Time.now()

                        else:			#We are far from the blob, so we continue to navigate towards the blob.
                        #While we are relatively far away from the blob (ie blob size is small relative to screen), we drive towards the center of the blob. 
                                if (blob_x_pos > 0.55): #If Blob is On the right side
                                        drive_msg.drive.steering_angle = .33 #Turn left
                                elif (blob_x_pos < 0.45): #If Blob is on the left side.
                                        drive_msg.drive.steering_angle = -0.33 #Turn right
                                else:	#If blob_x_pos (the x component of the blob's centroid) is in the middle 200 pixels, just move forward
                                        drive_msg.drive.steering_angle = 0

		if (Turning == True):
			if (turning_start_time + 3 > rospy.Time.now()):	#Keeps turning for three seconds (approx)
				drive_msg.drive.steering_angle = 0.33  if (blob_color.r == 255) else -0.33	#If the blob color is red, turn left, else if it is green, turn right.
			else:
				Turning = False #After turning process finishes, change Turning to False and Wall_Follow to true
				Wall_Follow = True		

                #publish the message
                self.pub_mov_func.publish(drive_msg)
		
if __name__=="__main__":
	rospy.init_node('ControlSystem')
	CC = MoveTowardImage()
	rospy.loginfo("Initializing Control System")
	rospy.spin()
