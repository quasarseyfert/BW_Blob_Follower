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
		self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.cbImage)
		self.blob_detect = rospy.Subscriber("/blob_detections", BlobDetections, self.MoveToBlob)	#blob_detect is an object that contains data about multiple blobs. The biggest blob is the one that has index 0. The same indices correspond to the same blob over the multiple data types in the message fields.


		#Talkers
		self.pub_mov_func = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=5)

		#Fields/Constants. Theses fields were used by 
		self.image_width = 0
		self.image_height = 0
		# State
		self.pid = {'p': 0.5, 'd': 1, 'i': 0}
		self.distance = {'desired': 0.5, 'last': 1, 'direction': 'right'}

		# Constants
		self.speed = 2
		self.collisionDistance = 1.5
		self.angles = {'left': 780, 'right': 300, 'sideRange': 10,
		               'frontRange': 15}

	def cbImage(self, data):
		self.img_width, self.img_height = cv.GetSize()
		
	def MoveToBlob(self,msg): #Moves to the largest blob
		drivemsg = AckermannDriveStamped() #Sets the drive messaged to the AckermannDriveStamped message type
		drivemsg.drive.speed = 2	#Sets a field of the drivemsg message.

				
		#Gets the position of the largest green blob. The first green blob will be the largest green blob because the blobs are sorted by size. (index 0 is biggest size)

		ele = ColorRGBA()
		ele.r = 0
		ele.g = 255
		largest_green_blob_index = self.blob_detect.colors.index(ele)

		blob_pos = self.blob_detec.locations[largest_green_bloc_index]  #Get's the position of the largest green block					
		blob_x_pos = blob_pos[0]
		blob_y_pos = blob_pos[1]
		
		biggest_blob_area = blob_detec.sizes[0] #The blob with index one is the biggest blob
		
		if (biggest_blob_area < 1000):	#Keep moving	
			if (blob_x_pos > self.img_width/2): 	#If Blob is On the right side
				drivemsg.drive.steering_angle = .33 #Turn left
			if (blob_x_pos < self.img_width/2): #If Blob is on the left side.
				drivemsg.drive.steering_angle = -0.33 #Turn right
			self.pub_mov_func.publish(drivemsg)
		else:	# if the contour is really big, we are close to the sign, so we do an emergency stop. We can delete this if we were to use the lidar.
			drivemsg.drive.speed = 0
			self.pub_mov_func.publish(drivemsg)

if __name__ == "__main__":#may want to set a hertz value
    rospy.init_node("ControlSystem")	#"ControlSystem" is the name of this instance of the program. We just made it the title of the file in this case.
    #bd = Echo()
    rospy.spin()





