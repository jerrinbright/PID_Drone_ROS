#!/usr/bin/env python

'''
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /roll_error             /pid_tuning_altitude
        /pitch_error            /pid_tuning_pitch
        /yaw_error              /pid_tuning_roll
        /edrone/pwm             /edrone/imu/data
                                /edrone/drone_command

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf


class Edrone():
    def __init__(self):
        #defing and assigning the varibles with default vales
        rospy.init_node('attitude_controller')
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
        self.drone_orientation_euler = [0.0 , 0.0 , 0.0] #[roll , pitch ,yaw]
        self.error = [0.0,0.0,0.0]
        self.setpoint_cmd = [0.0,0.0,0.0,0.0]
        self.PID_PV = [0.0,0.0,0.0]
        self.sample_time = 0.060
        self.Kp = [4.24562,4.24562,0]
        self.Kd = [4.5,4.5,0.0]
        self.Ki = [0.0,0.0,0.0]
        self.setpoint_base_speed = 0.0
        self.setpoint_euler = [0.0,0.0,0.0]
        self.Iterm = [0.0,0.0,0.0]
        self.prev_values = [0.0,0.0,0.0]
        self.max_values = [1024.0,1024.0,1024.0,1024.0]
        self.min_values = [0.0,0.0,0.0,0.0]
        self.PV_roll= 0.0
        self.PV_pitch= 0.0
        self.PV_Yaw= 0.0

        self.RollErrorPub = Float32()
        self.pitchErrorPub = Float32()
        self.ZeroError = Float32()
        self.ZeroError = 0.0
        self.YawErrorPub = Float32()
        self.pi = 3.14159265359
        #defing the prop_speed msg
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0
        #defing the publisher
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        self.roll_error=rospy.Publisher('/roll_error',Float32,queue_size=1)
        self.pitch_error=rospy.Publisher('/pitch_error',Float32,queue_size=1)
        self.yaw_error=rospy.Publisher('/yaw_error',Float32,queue_size=1)
        self.zero_error=rospy.Publisher('/zero_error',Float32,queue_size=1)
        

        #defing the subscriber
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        '''rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_yaw',PidTune,self.yaw_set_pid)'''
    #defing the required callbacks   
    def imu_callback(self, msg):
        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w

    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.setpoint_cmd[3] = msg.rcThrottle

    '''def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.002 
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 0.002

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.002  
        self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 0.002

    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp * 0.002
        self.Ki[2] = yaw.Ki * 0.008
        self.Kd[2] = yaw.Kd * 0.002'''
    #defing the pid function
    def pid(self):
        #defing the setpoint variables
        (self.drone_orientation_euler[1], self.drone_orientation_euler[0], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])
        self.setpoint_euler[0] = (self.setpoint_cmd[0] * 0.02) - 30
        self.setpoint_euler[1] = (self.setpoint_cmd[1] * 0.02) - 30
        self.setpoint_euler[2] = (self.setpoint_cmd[2] * 0.02) - 30
        self.throttle = (self.setpoint_cmd[3] * 1.024) - 1024

        # Calculating the error
        self.error[0] = self.setpoint_euler[0] - (self.drone_orientation_euler[0]*(180/self.pi))
        self.error[1] = self.setpoint_euler[1] - (self.drone_orientation_euler[1]*(180/self.pi))
        self.error[2] = self.setpoint_euler[2] - (self.drone_orientation_euler[2]*(180/self.pi))

        #calculating the Iterm
        self.Iterm[0] = self.Iterm[0] + self.error[0]
        self.Iterm[1] = self.Iterm[1] + self.error[1]
        self.Iterm[2] = self.Iterm[2] + self.error[2]

        # Calculating pid process varibal
        self.PV_roll = (self.Kp[0] * self.error[0]) + (self.Ki[0] * self.Iterm[0]) + ((self.Kd[0] * (self.error[0] - self.prev_values[0]))/self.sample_time)
        self.PV_pitch = (self.Kp[1] * self.error[1]) + (self.Ki[1] * self.Iterm[1]) + ((self.Kd[1] * (self.error[1] - self.prev_values[1]))/self.sample_time)
        self.PV_Yaw = (self.Kp[2] * self.error[2]) + (self.Ki[2] * self.Iterm[2]) + ((self.Kd[2] * (self.error[2] - self.prev_values[2]))/self.sample_time)

        #defing the prev error values
        self.prev_values[0] = self.error[0]
        self.prev_values[1] = self.error[1]
        self.prev_values[2] = self.error[2]

        # Giving pwm values wit
        self.pwm_cmd.prop1 = self.throttle - self.PV_roll + self.PV_pitch - self.PV_Yaw
        self.pwm_cmd.prop2 = self.throttle - self.PV_roll - self.PV_pitch + self.PV_Yaw
        self.pwm_cmd.prop3 = self.throttle + self.PV_roll - self.PV_pitch - self.PV_Yaw
        self.pwm_cmd.prop4 = self.throttle + self.PV_roll + self.PV_pitch + self.PV_Yaw
        #checking for min and amx vales
        if self.pwm_cmd.prop1 > self.max_values[0]:
            self.pwm_cmd.prop1 = self.max_values[0]
        elif self.pwm_cmd.prop1 < self.min_values[0]:
            self.pwm_cmd.prop1 = self.min_values[0]


        if self.pwm_cmd.prop2 > self.max_values[1]:
            self.pwm_cmd.prop2 = self.max_values[1]
        elif self.pwm_cmd.prop2 < self.min_values[1]:
            self.pwm_cmd.prop2 = self.min_values[1]
        
        if self.pwm_cmd.prop3 > self.max_values[2]:
            self.pwm_cmd.prop3 = self.max_values[2]
        elif self.pwm_cmd.prop3 < self.min_values[2]:
            self.pwm_cmd.prop3 = self.min_values[2]
        

        if self.pwm_cmd.prop4 > self.max_values[3]:
            self.pwm_cmd.prop4 = self.max_values[3]
        elif self.pwm_cmd.prop4 < self.min_values[3]:
            self.pwm_cmd.prop4 = self.min_values[3]
        

        #pub the pwm_cmd msg
        self.pwm_pub.publish(self.pwm_cmd)
        #print(self.pwm_cmd)

#defing the main func
if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)
    while not rospy.is_shutdown():
        try:
            e_drone.pid()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException : pass 