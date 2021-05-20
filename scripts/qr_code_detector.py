#!/usr/bin/env python


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from pyzbar.pyzbar import decode

class image_proc():
	def __init__(self):
		rospy.init_node('barcode_test') #Initialise rosnode 
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		self.i = 0
		self.x=[0.0]
		self.s = [0.0,0.0,0.0]



	# Callback function of amera topic
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image

			result=decode(self.img)
			for i in result:
				self.x=(i.data.decode("utf-8"))
			print(self.x)
		except CvBridgeError as e:
			print(e)
			return


if __name__ == '__main__':
    image_proc_obj = image_proc()
    #image_proc_obj.qr()
    rospy.spin()