#!/usr/bin/env python


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Imu, NavSatFix , LaserScan
from vitarana_drone.srv import Gripper
from std_msgs.msg import String
import ast
from std_msgs.msg import Float32

class edrone():
	def __init__(self):
		rospy.init_node('task2')
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		self.i = 0#define allt hthe necessary varibles
		self.cc=[0.0]
		self.s = [0.0,0.0,0.0]
		self.loop_for_grip = 1
		self.val = False
		self.now = rospy.Time.now()
		self.dist =[0.0,0.0,0.0]#lFR
		self.coordinates = [0.0,0.0,0.0]
		self.pickupflag=0
		self.lat = Float32()
		self.lat=0.0
		self.lon = Float32()
		self.lon=0.0
		self.alt = Float32()
		self.alt=0.0
		#defne bler and subscriber
		self.lat_pub=rospy.Publisher('/lat',Float32,queue_size=1)
		self.lon_pub=rospy.Publisher('/lon',Float32,queue_size=1)
		self.alt_pub =rospy.Publisher('/alt',Float32,queue_size=1)
		rospy.Subscriber('/edrone/gripper_check',String,self.check)
		rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.rangecall)
		rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
		self.pickup_coordinates =[19.0009248718,71.9998318945,27.0]
		self.flagp=0
		self.x = False
		self.gripflag = 0
		self.fixcoord = [0.0]
		self.tarlat = 0.0
		self.tarlon = 0.0
		self.taralt = 0.0
		self.tarlatflag = 0 
		self.latlocalflag = 0

	#grippe callback
	def check(self,data):
		self.val = ast.literal_eval(data.data)
		value = self.val
		# rospy.wait_for_service("edrone/activate_gripper")
		if value == True  and self.loop_for_grip==1:
			grip = rospy.ServiceProxy("edrone/activate_gripper",Gripper)
			self.x = grip.call(True)
			print(self.x)
			self.loop_for_grip=0
			self.gripflag = 1
	#rage fder cal back
	def rangecall(self,msg):
		self.dist[0] = msg.ranges[3]
		self.dist[1] = msg.ranges[0]
		self.dist[2] = msg.ranges[1]
		#print(self.dist)
	def gps_callback(self, msg):#gps caback
		self.coordinates[0] = msg.latitude
		self.coordinates[1] = msg.longitude
		self.coordinates[2] = msg.altitude
	#qrcode calbback
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
			for barcode in decode(self.img):
				mycode = barcode.data.decode('UTF-8')
				encoded_data=mycode.encode('utf-8')
				self.cc=list(encoded_data.split(','))
				
		except CvBridgeError as e:
			print(e)
			return


	#pickup the box
	def pickup(self):
		if self.pickupflag == 0 :
			if self.flagp == 0:
				self.lat=self.pickup_coordinates[0]
				self.lon=self.pickup_coordinates[1]
				self.alt=self.pickup_coordinates[2]				
			if self.coordinates[2] > 26.4 and self.coordinates[2]<27.2:
				self.pickup_coordinates[0]=19.0007046575
				self.pickup_coordinates[1]=71.9998955286
				self.pickup_coordinates[2]=24.0
				self.lat=self.pickup_coordinates[0]
				self.lon=self.pickup_coordinates[1]
				self.alt=self.pickup_coordinates[2]
				self.flagp = 1
			if self.coordinates[0] > 19.0007044575 and self.coordinates[0]<19.000708575:
				self.pickup_coordinates[0]=19.0007046575
				self.pickup_coordinates[1]=71.9998955286
				self.pickup_coordinates[2]=22.1599967919
				self.lat=self.pickup_coordinates[0]
				self.lon=self.pickup_coordinates[1]
				self.alt=self.pickup_coordinates[2]
				self.flagp = 2
			if self.coordinates[2] > 20.4 and self.coordinates[2]<22.5 and self.gripflag == 1:
				self.pickup_coordinates[0]=19.0007046575
				self.pickup_coordinates[1]=71.9998955286
				self.pickup_coordinates[2]=25
				self.lat=self.pickup_coordinates[0]
				self.lon=self.pickup_coordinates[1]
				self.alt=self.pickup_coordinates[2]
				self.flagp = 3
			if self.coordinates[2] > 24.8 and self.coordinates[2]<25.5 and self.gripflag == 1:
				self.pickupflag = 1
				print("pickup completed")
			self.lat_pub.publish(self.lat)
			self.lon_pub.publish(self.lon)
			self.alt_pub.publish(self.alt)
	def gotodrop(self):#bug 0 algo
		if self.pickupflag ==1:
			self.fixcoord = self.cc
			self.tarlat = float(self.fixcoord[0])
			self.tarlon = float(self.fixcoord[1])
			self.taralt = float(self.fixcoord[2])
			
			while self.tarlatflag == 0:
				if self.dist[0] > 1 and self.dist[1] > 1 and self.dist[2] > 1:
					#front
					self.lat = self.lat + 1
				elif self.dist[0] > 1 and self.dist[1] > 1 and self.dist[2] < 1:
					#front
					self.lat = self.lat + 1
				elif self.dist[0] > 1 and self.dist[1] < 1 and self.dist[2] > 1:
					#right/left default: right
					self.lon = self.lon + 1
				elif self.dist[0] > 1 and self.dist[1] < 1 and self.dist[2] < 1:
					#right
					self.lon = self.lon + 1
				elif self.dist[0] < 1 and self.dist[1] > 1 and self.dist[2] > 1:
					#front
				elif self.dist[0] < 1 and self.dist[1] > 1 and self.dist[2] < 1:
					#front
					self.lat = self.lat + 1
				elif self.dist[0] < 1 and self.dist[1] < 1 and self.dist[2] > 1:
					#left
					self.lon = self.lon - 1
				else:
					if self.tarlat != self.coordinates[0]:
						if self.tarlat > self.coordinates[0]:
							self.lat = self.lat + 1
						elif self.tarlat < self.coordinates[0]:
							self.lat = self.lat - 1

						if self.tarlon != self.coordinates[1]:
							if self.tarlon > self.coordinates[1]:
								self.lon = self.lon + 1
						elif self.tarlon < self.coordinates[1]:
							self.lon = self.lon - 1

						if self.tarlon != self.coordinates[2]:
							if self.tarlon > self.coordinates[2]:
								self.alt = self.alt + 1
						elif self.tarlon < self.coordinates[2]:
							self.alt = self.alt - 1
				if self.lat == self.tarlat and self.lon == self.tarlon and self.alt == self.taralt:
					self.tarlatflag == 1
				
















if __name__ == '__main__':

	e_drone = edrone()
	r = rospy.Rate(1/0.060)  
	while not rospy.is_shutdown():
		try:
			e_drone.pickup()
			r.sleep()
		except rospy.exceptions.ROSTimeMovedBackwardsException: pass
