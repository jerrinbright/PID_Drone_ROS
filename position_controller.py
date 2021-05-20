#!/usr/bin/env python
#Declare all the libraries
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32
import rospy
import time
import tf

#declare a class name Edrone
class Edrone():
    
    def __init__(self):#def constructor
        rospy.init_node('position_controller')  

        
        self.dronecmd = edrone_cmd()#define the edrone_cmd msg for publishing
        self.dronecmd.rcRoll = 1500#declare the values for roll pitch yaw throttle
        self.dronecmd.rcPitch = 1500
        self.dronecmd.rcYaw = 1500
        self.dronecmd.rcThrottle = 0
        self.now = rospy.Time.now()#Declare a variable to store the time of the start of exefcution
        self.coordinates = [0.0,0.0,0.0]#variable for storing the current coordinates
        self.target = [19.0,72.0,3.0]#setpoint 1
        self.setpoint2=[19.0000451704,72.0,3.0]#setpoint 2
        self.setpoint3= [19.0000451704,72.0,0.0]#setpoint 3
        self.Kp = [40000000,40000000,10.0]#5.0 Kp values declare
        self.Ki = [0.0,0.0,0.0]#ki valus declare
        self.Kd = [50000000,50000000, 1250]#2513 kd alus declare
        self.error = [0, 0, 0]#error val
        self.prev_error = [0, 0, 0]#prev error vaal
        self.Iterm = [0, 0, 0]#item val
        self.PV_PID = [0.0,0.0,0.0]#process varible declare
        self.flag=0#flag varible declare
        #set maximum and minimum value
        self.min_value = [1000, 1000, 1000]
        self.max_value = [2000, 2000, 2000]
        self.sample_time = 0.060  #declare sample time
        #declare publisher and subscriber
        self.dronecmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        
    #def gps callback for subscriber
    def gps_callback(self, msg):
        self.coordinates[0] = msg.latitude
        self.coordinates[1] = msg.longitude
        self.coordinates[2] = msg.altitude

        #define the PID func
    def pid(self):
    	#def the allowance for target coordinates
    	if self.coordinates[2] > 2.8 and self.coordinates[2]<3.2 and self.flag==0 and rospy.Time.now()> self.now + rospy.Duration.from_sec(25) :
    		print('target1 reached')
    		#print(self.coordinates)
    		self.flag=1
    		self.target[0]=self.setpoint2[0]
    		self.target[1]=self.setpoint2[1]
    		self.target[2]=self.setpoint2[2]

    	elif self.coordinates[0] >19.0000406534 and self.coordinates[0]<19.0000496874 and self.flag==1  and rospy.Time.now()> self.now + rospy.Duration.from_sec(32) :
    		print('target 2 reached')
    		self.flag=2
    		#print(self.coordinates)
    		self.target[0]=self.setpoint3[0]
    		self.target[1]=self.setpoint3[1]
    		self.target[2]=self.setpoint3[2]
    	elif self.coordinates[2]>0.1 and self.coordinates[2]<0.4 and self.flag==2:
    		print('Task - completed')
    		self.flag=3


    	#calculating the error
        self.error[0]  =self.target[0] - self.coordinates[0]
        self.error[1] = self.target[1] - self.coordinates[1]
        self.error[2] = self.target[2] - self.coordinates[2]

        #calculating the Iterm
        self.Iterm[0] = self.Iterm[0] + self.error[0]
        self.Iterm[1] = self.Iterm[1] + self.error[1]
        self.Iterm[2] = self.Iterm[2] + self.error[2]

        #calculate the process variable
        self.PV_PID[0] = (self.Kp[0] * self.error[0]) + (self.Ki[0] * self.Iterm[0]) + ((self.Kd[0] * (self.error[0] - self.prev_error[0]))/self.sample_time)
        self.PV_PID[1] = (self.Kp[1] * self.error[1]) + (self.Ki[1] * self.Iterm[1]) + ((self.Kd[1] * (self.error[1] - self.prev_error[1]))/self.sample_time)
        self.PV_PID[2] = (self.Kp[2] * self.error[2]) + (self.Ki[2] * self.Iterm[2]) + ((self.Kd[2] * (self.error[2] - self.prev_error[2])))

        #definrthe prev erroer

        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]
        #adding the process varibles to setpoints
        self.dronecmd.rcRoll = 1500 + self.PV_PID[0]
        self.dronecmd.rcPitch = 1500 + self.PV_PID[1]
        self.dronecmd.rcYaw = 1500
        self.dronecmd.rcThrottle = 1500 + self.PV_PID[2]
        #checking the vlaus and defining the max and min
        if self.dronecmd.rcRoll > self.max_value[0]:
            self.dronecmd.rcRoll = self.max_value[0]
        elif self.dronecmd.rcRoll < self.min_value[0]:
            self.dronecmd.rcRoll = self.min_value[0]
        else:
            self.dronecmd.rcRoll = self.dronecmd.rcRoll

        if self.dronecmd.rcPitch > self.max_value[1]:
            self.dronecmd.rcPitch = self.max_value[1]
        elif self.dronecmd.rcPitch < self.min_value[1]:
            self.dronecmd.rcPitch = self.min_value[1]
        else:
            self.dronecmd.rcPitch = self.dronecmd.rcPitch

        if self.dronecmd.rcThrottle > self.max_value[2]:
            self.dronecmd.rcThrottle = self.max_value[2]
        elif self.dronecmd.rcThrottle < self.min_value[2]:
            self.dronecmd.rcThrottle = self.min_value[2]
        else:
            self.dronecmd.rcThrottle = self.dronecmd.rcThrottle
        self.dronecmd_pub.publish(self.dronecmd)#Publishing the dronecmd msg
        

#defing thr main func
if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  
    while not rospy.is_shutdown():
        try:
            e_drone.pid()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException: pass





