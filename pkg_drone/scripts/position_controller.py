#!/usr/bin/env python

# Made by  Team ID - VD_0142

# Importing the required libraries
import rospy
from sensor_msgs.msg import *
from pkg_drone.msg import *



class Position_Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')

        # Publisher: to publish values to attitude controller
        self.drone_command = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

        # Subscriber: to subscribe to location of drone
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)


        # [latitude, longitude, altitude]
        self.set_values = [[19.0, 72.0, 3.0], [19.0000451704, 72.0, 3.0], [19.0000451704, 72.0, 0.0]]  # set values for drone
        self.edrone_pos = [19.0, 72.0, 0.0]  # Current Position Of Drone

        # PID Gains [x error (pitch), y error(roll)]
        self.kp = [220 * .0001, 0.0220 * .0001]
        self.ki = [0.0, 0.0]
        self.kd = [393 * .03, 393 * .03]

        # [for x, for y, for z]
        self.error = [0.0, 0.0, 0.0]
        self.err_sum = [0.0, 0.0]
        self.last_err = [0.0, 0.0]
        self.dErr = [0.0, 0.0]

        # SampleTime for PID values
        self.sample_time = 1.0/200.0

        # Counters
        self.i = 0
        self.j = 0

        # Creating Variable to Publish Values 
        self.position = edrone_cmd()
        self.position.rcRoll = 1500.0
        self.position.rcPitch = 1500.0
        self.position.rcYaw = 1500.0
        self.position.rcThrottle = 1500.0
        self.drone_command.publish(self.position)

    # Callback function to get current position of drone
    def gps_callback(self, msg):
        self.edrone_pos[0] = msg.latitude
        self.edrone_pos[1] = msg.longitude
        self.edrone_pos[2] = msg.altitude
    
    # Main Function
    def path(self):
        """docstring for main functions"""
        self.error[0] = (self.set_values[self.i][0] - self.edrone_pos[0]) * 1000000  # multiplying factor, since error is very small
        self.error[1] = (self.set_values[self.i][1] - self.edrone_pos[1]) * 1000000  # multiplying factor, since error is very small
        self.error[2] = self.set_values[self.i][2] - self.edrone_pos[2]

        self.err_sum[0] = self.err_sum[0] + self.error[0]
        self.err_sum[1] = self.err_sum[1] + self.error[1]
        
        if not self.error[0] - self.last_err[0] == 0 and (self.error[0] - self.last_err[0] < 0.1):
            self.dErr[0] = (self.error[0] - self.last_err[0])
        
        if not self.error[1] - self.last_err[1] == 0 and (self.error[0] - self.last_err[0] < 0.1):
            self.dErr[1] = (self.error[1] - self.last_err[1])
        # Here condition (last_err < 0.1) is considered to stop misbehaving of drone during abrupt change
        

        # Adding Output Values
        self.position.rcPitch = max(min(1500.0 + (self.kp[0] * self.error[0] + self.ki[0] * self.err_sum[0] + self.kd[0] * self.dErr[0]), 2000), 1000)
        self.position.rcRoll = max(min(1500.0 + (self.kp[1] * self.error[1] + self.ki[1] * self.err_sum[1] + self.kd[1] * self.dErr[1]), 2000), 1000)
        self.position.rcYaw = 1500.0
        self.position.rcThrottle = max(min(1500 + self.error[2] * 100, 2000), 1000)
        
        # Publishsing Output Values
        self.drone_command.publish(self.position)


        self.last_err[0] = self.error[0]
        self.last_err[1] = self.error[1]


        # Checking conditions of Check Points
        self.i = self.j//400

        if self.i == 0:
            if round(self.error[2], 2) == 0 or self.j > 0:
                self.j = self.j + 1
        
        if self.i == 1:
            if round(self.error[0], 3) == 0 or self.j > 400:
                self.j = self.j + 1
        
        if self.i == 2:
            if round(self.error[2], 2) < 1.6 or self.j > 800:
                self.j = self.j + 1
        # Here j is considered so as to let the drone stable on one position
        # before moving towards next Check Point




if __name__ == '__main__':

    e_drone = Position_Edrone()
    r = rospy.Rate(1/e_drone.sample_time)
    while not (rospy.is_shutdown() or e_drone.i > 2):
        e_drone.path()
        r.sleep()