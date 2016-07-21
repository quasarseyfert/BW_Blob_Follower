#!/usr/bin/python
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy


class Joy_Reader:
	def __init__(self):
		rospy.Subscriber("/vesc/joy", Joy, self.joy_callback)
		
		self.joy_pub = rospy.Publisher("racecar/JoyLRSelec", Bool, queue_size = 1)
		
	def joy_callback(self, JoyMsg):
		buttons = JoyMsg.buttons
		if buttons[2] != 0:
			self.joy_pub.publish(1)
		elif buttons[1] != 0:
			self.joy_pub.publish(0)
		else:
			return
if __name__ == '__main__':
	rospy.init_node("Joy_Reader")
	node = Joy_Reader()
	rospy.spin()
