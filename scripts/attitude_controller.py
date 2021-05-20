#!/usr/bin/env python

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32
import rospy
import time
import tf


class Edrone():
    
    def __init__(self):
        rospy.init_node('position_controller')  

        # Format for drone_command
        self.dronecmd = edrone_cmd()
        self.dronecmd.rcRoll = 1500
        self.dronecmd.rcPitch = 1500
        self.dronecmd.rcYaw = 1500
        self.dronecmd.rcThrottle = 0

        self.coordinates = [0.0,0.0,0.0]
        self.target = [19.0000451704,72.0,3.0]
        self.Kp = [40000000,40000000,5.0]
        self.Ki = [0.0,0.0,0.0]
        self.Kd = [50000000,50000000, 2513]
        self.error = [0, 0, 0]
        self.prev_error = [0, 0, 0]
        self.error_sum = [0, 0, 0]
        self.PV_PID = [0.0,0.0,0.0]

        self.min_value = [1000, 1000, 1000]
        self.max_value = [2000, 2000, 2000]
        self.sample_time = 0.060  
        self.dronecmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        

    def gps_callback(self, msg):
        self.coordinates[0] = msg.latitude
        self.coordinates[1] = msg.longitude
        self.coordinates[2] = msg.altitude


    def pid(self):

        self.error[0]  =self.target[0] - self.coordinates[0]
        self.error[1] = self.target[1] - self.coordinates[1]
        self.error[2] = self.target[2] - self.coordinates[2]


        self.error_sum[0] = self.error_sum[0] + self.error[0]
        self.error_sum[1] = self.error_sum[1] + self.error[1]
        self.error_sum[2] = self.error_sum[2] + self.error[2]


        self.PV_PID[0] = (self.Kp[0] * self.error[0]) + (self.Ki[0] * self.error_sum[0]) + ((self.Kd[0] * (self.error[0] - self.prev_error[0]))/self.sample_time)
        self.PV_PID[1] = (self.Kp[1] * self.error[1]) + (self.Ki[1] * self.error_sum[1]) + ((self.Kd[1] * (self.error[1] - self.prev_error[1]))/self.sample_time)
        self.PV_PID[2] = (self.Kp[2] * self.error[2]) + (self.Ki[2] * self.error_sum[2]) + ((self.Kd[2] * (self.error[2] - self.prev_error[2])))



        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]
        self.dronecmd.rcRoll = 1500 + self.PV_PID[0]
        self.dronecmd.rcPitch = 1500 + self.PV_PID[1]
        self.dronecmd.rcYaw = 1500
        self.dronecmd.rcThrottle = 1500 + self.PV_PID[2]

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
        self.dronecmd_pub.publish(self.dronecmd)


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  
    while not rospy.is_shutdown():
        try:
            e_drone.pid()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException: pass





