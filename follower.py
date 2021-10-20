#!/usr/bin/env python3

"""
Start ROS node to publish linear and angular velocities to mymobibot in order to perform wall following.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t

# from tf.transformations import euler_from_quaternion
# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

def quaternion_to_euler(w, x, y, z):
    """Converts quaternions with components w, x, y, z into a tuple (roll, pitch, yaw)"""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1, np.sign(sinp) * np.pi / 2, np.arcsin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class mymobibot_follower():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):

        # linear and angular velocity
        self.velocity = Twist()
        # joints' states
        self.joint_states = JointState()
        # Sensors
        self.imu = Imu()
        self.imu_yaw = 0.0 # (-pi, pi]
        self.sonar_F = Range()
        self.sonar_FL = Range()
        self.sonar_FR = Range()
        self.sonar_L = Range()
        self.sonar_R = Range()

        # ROS SETUP
        # Initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.velocity_pub = rospy.Publisher('/mymobibot/cmd_vel', Twist, queue_size=1)
        self.joint_states_sub = rospy.Subscriber('/mymobibot/joint_states', JointState, self.joint_states_callback, queue_size=1)
        # Sensors
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)
        self.sonar_front_sub = rospy.Subscriber('/sensor/sonar_F', Range, self.sonar_front_callback, queue_size=1)
        self.sonar_frontleft_sub = rospy.Subscriber('/sensor/sonar_FL', Range, self.sonar_frontleft_callback, queue_size=1)
        self.sonar_frontright_sub = rospy.Subscriber('/sensor/sonar_FR', Range, self.sonar_frontright_callback, queue_size=1)
        self.sonar_left_sub = rospy.Subscriber('/sensor/sonar_L', Range, self.sonar_left_callback, queue_size=1)
        self.sonar_right_sub = rospy.Subscriber('/sensor/sonar_R', Range, self.sonar_right_callback, queue_size=1)

        # New publishers needed for plotting desired variables
        self.R_sonar_err = rospy.Publisher('/R_sonar_error', Float64, queue_size=100)
        self.FR_sonar_err = rospy.Publisher('/FR_sonar_error', Float64, queue_size=100)
        self.F_sonar = rospy.Publisher('/F_sonar', Float64, queue_size=100)

        self.x_vel = rospy.Publisher('/linear_x_velocity', Float64, queue_size=100)
        self.z_angvel = rospy.Publisher('/angulaz_z_velocity', Float64, queue_size=100)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of the left wheel is stored in :: self.joint_states.position[0])
        # (e.g. the angular velocity of the right wheel is stored in :: self.joint_states.velocity[1])

    def imu_callback(self, msg):
        # ROS callback to get the /imu

        self.imu = msg
        # (e.g. the orientation of the robot wrt the global frome is stored in :: self.imu.orientation)
        # (e.g. the angular velocity of the robot wrt its frome is stored in :: self.imu.angular_velocity)
        # (e.g. the linear acceleration of the robot wrt its frome is stored in :: self.imu.linear_acceleration)

        #quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        #(roll, pitch, self.imu_yaw) = euler_from_quaternion(quaternion)
        (roll, pitch, self.imu_yaw) = quaternion_to_euler(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)

    def sonar_front_callback(self, msg):
        # ROS callback to get the /sensor/sonar_F

        self.sonar_F = msg
        # (e.g. the distance from sonar_front to an obstacle is stored in :: self.sonar_F.range)

    def sonar_frontleft_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FL

        self.sonar_FL = msg
        # (e.g. the distance from sonar_frontleft to an obstacle is stored in :: self.sonar_FL.range)

    def sonar_frontright_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FR

        self.sonar_FR = msg
        # (e.g. the distance from sonar_frontright to an obstacle is stored in :: self.sonar_FR.range)

    def sonar_left_callback(self, msg):
        # ROS callback to get the /sensor/sonar_L

        self.sonar_L = msg
        # (e.g. the distance from sonar_left to an obstacle is stored in :: self.sonar_L.range)

    def sonar_right_callback(self, msg):
        # ROS callback to get the /sensor/sonar_R

        self.sonar_R = msg
        # (e.g. the distance from sonar_right to an obstacle is stored in :: self.sonar_R.range)

    def publish(self):

        # set configuration
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        print("The system is ready to execute your algorithm...")

        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()
        mode = 1 

        while not rospy.is_shutdown():

            # Renaming given variables (only counter-clockwise needed ones)
            front_sensor = self.sonar_F.range 
            frontR_sensor = self.sonar_FR.range 
            right_sensor = self.sonar_R.range

            # Calculate time interval (in case is needed)
            time_prev = time_now
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.to_nsec()
            dt = (time_now - time_prev)/1e9

            '''
                Mode 1 -> Starting route until the vehicle reaches the wall
                Mode 2 -> Left turn without controller help
                Mode 3 -> Following the wall using PD controller
            '''
            # Gains for the PD controller
            Kp = 30
            Kd = 5
            # Hypotenuse from small triangle with sensor and robot
            d0 = np.sqrt(2 * 0.018**2)

            if (mode == 1):
                self.velocity.linear.x = 0.5
                
                if (front_sensor < 0.4):
                    mode = 2
            
            elif (mode == 2):
                self.velocity.angular.z = - 2
                self.velocity.linear.x = 0.0

                if (front_sensor > 0.4):
                    # Initialisations for PD controller
                    FR_error = 0
                    R_error = 0
                    mode = 3 

            else:
                previousFR_error = FR_error
                previousR_error = R_error            

                # Calculating proportional errors via sensors
                # Demanded/safe distance from the wall chosen
                dist = 0.2       
                R_error = dist - right_sensor

                # Geometrically found calculations so as robot to be parallel to the wall

                diagonal_dist = dist / cos(pi/4) + d0 
                FR_error =  diagonal_dist - frontR_sensor
                
                prop = FR_error + R_error

                # Calculating derivative errors
                FR_der = FR_error - previousFR_error
                R_der = R_error - previousR_error

                deriv = (FR_der + R_der) / (dt + 10**(-9))

                # Set the right velocities for mode 3 to fix position 
                self.velocity.linear.x = 0.5
                self.velocity.angular.z = -max(min(Kp*prop + Kd*deriv, 0.5), -0.5)
                
                # In case robot has to turn to avoid wall return to mode 2 
                if (front_sensor < 0.3):
                    mode = 2

            # Publish the new joint's angular positions
            self.velocity_pub.publish(self.velocity)

            # Publish for plotting
            self.R_sonar_err.publish(right_sensor)
            self.FR_sonar_err.publish(frontR_sensor)
            self.F_sonar.publish(front_sensor)
            self.x_vel.publish(self.velocity.linear.x)
            self.z_angvel.publish(self.velocity.angular.z)


            self.pub_rate.sleep()

    def turn_off(self):
        pass

def follower_py():
    # Starts a new node
    rospy.init_node('follower_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    follower = mymobibot_follower(rate)
    rospy.on_shutdown(follower.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        follower_py()
    except rospy.ROSInterruptException:
        pass
