#!/usr/bin/env python

# Made by  Team ID - VD_0142

# Importing the required libraries
import tf
import rospy
from sensor_msgs.msg import *
from pkg_drone.msg import *



class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('attitude_controller')  # initializing ros node with name drone_control

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [1500.0, 1500.0, 1500.0, 1500.0]

        # The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]


        # Declaring pwm_cmd of message type prop_speed and initializing values
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0
 
        # initial setting of kp, kd and ki for [roll, pitch, yaw, throttle]
        self.kp = [500 * 0.06, 500 * 0.06, 0.0, 424 * 0.5]
        self.ki = [0.0, 0.0, 0.0, 10000 * .00005]
        self.kd = [3600 * 0.3, 3600 * 0.3, 0.0, 1020  * 10]

        # Other initializations
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        self.altitude_error = 0.0
        self.error = [0.0, 0.0, 0.0, 0.0]
        self.err_sum = [0.0, 0.0, 0.0, 0.0]
        self.last_err = [0.0, 0.0, 0.0, 0.0]
        self.dErr = [0.0, 0.0, 0.0, 0.0]
        
        self.Output = [0.0, 0.0, 0.0, 0.0]

        # This is the sample time
        self.sample_time = 0.005


        # Publisher: For publishsing Propeller speeds
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)


        # Subscribing to /drone_command, imu/data
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)


    # CallBack Functions
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

    # Main function to be excecuted
    def pid(self):
        """ docstring for main functions"""
        # Converting quaternion to euler angles
        (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])
        
        # Converting the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll axis
        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
        self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30

        # Converting the range from 1000 to 2000 in the range of -5m to 5m for altitude error
        self.altitude_error = self.setpoint_cmd[3] * .01 - 15

        # computing error
        self.error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0] 
        self.error[1] = self.setpoint_euler[1] - self.drone_orientation_euler[1] 
        self.error[2] = self.setpoint_euler[2] - self.drone_orientation_euler[2] 
        self.error[3] = self.altitude_error

        # computing err_sum
        if not round(self.error[0], 3) == 0:
            self.err_sum[0] = self.err_sum[0] + self.error[0] 
        if not round(self.error[1], 3) == 0:
            self.err_sum[1] = self.err_sum[1] + self.error[1]
        if not round(self.error[2], 3) == 0:
            self.err_sum[2] = self.err_sum[2] + self.error[2] 
        if not round(self.error[3], 3) == 0:
            self.err_sum[3] = self.err_sum[3] + self.error[3]

        # computing dErr
        if not self.error[0] - self.last_err[0] == 0:
            self.dErr[0] = (self.error[0] - self.last_err[0])
        if not self.error[1] - self.last_err[1] == 0:
            self.dErr[1] = (self.error[1] - self.last_err[1])
        if not self.error[2] - self.last_err[2] == 0:
            self.dErr[2] = (self.error[2] - self.last_err[2])
        if not self.error[3] - self.last_err[3] == 0:
            self.dErr[3] = (self.error[3] - self.last_err[3])
  
        # Compute PID Output
        self.out_roll = self.kp[0] * self.error[0] + self.ki[0] * self.err_sum[0] + self.kd[0] * self.dErr[0]
        self.out_pitch = self.kp[1] * self.error[1] + self.ki[1] * self.err_sum[1] + self.kd[1] * self.dErr[1]
        self.out_yaw = self.kp[2] * self.error[2] + self.ki[2] * self.err_sum[2] + self.kd[2] * self.dErr[2]
        self.out_altitude = self.kp[3] * self.error[3] + self.ki[3] * self.err_sum[3] + self.kd[3] * self.dErr[3]
    
        self.Output = [self.out_roll, self.out_pitch, self.out_yaw, self.out_altitude]   
        
        # Updating propellers speed with error
        self.pwm_cmd.prop1 = max(min(self.Output[3] + self.Output[0] - self.Output[1] + self.Output[2], 1023), 0) 
        self.pwm_cmd.prop2 = max(min(self.Output[3] - self.Output[0] - self.Output[1] - self.Output[2], 1023), 0) 
        self.pwm_cmd.prop3 = max(min(self.Output[3] - self.Output[0] + self.Output[1] + self.Output[2], 1023), 0) 
        self.pwm_cmd.prop4 = max(min(self.Output[3] + self.Output[0] + self.Output[1] - self.Output[2], 1023), 0) 

        # Publishing the propeller speed
        self.pwm_pub.publish(self.pwm_cmd)
        
        # updating the last error with recent error
        self.last_err[0] = self.error[0]
        self.last_err[1] = self.error[1]
        self.last_err[2] = self.error[2]
        self.last_err[3] = self.error[3]



if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)   # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()