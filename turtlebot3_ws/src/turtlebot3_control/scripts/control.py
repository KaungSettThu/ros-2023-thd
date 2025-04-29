#!/usr/bin/env/python3

import rospy

import numpy as np
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

from math import atan2, sin, cos, sqrt, pi



class TurtleBot3Controller:
    def __init__(self):

        # 1. initialize the ROS node

        rospy.init_node('my_turtlebot3_control')
        rospy.loginfo("Turtlebot 3 Sigwart et. al. Controller Started...")

        # 2. set the initial position and orientation

        self.x = 0
        self.y = 0
        self.yaw = 0

        #3. set the goal of the robot

        self.x_goal = 8.0
        self.y_goal = 2.0
        self.yaw_goal = pi / 2

        # 4. define the control gains of the robot

        self.k_rho = 0.2
        self.k_alpha = 0.6      # k_alpha > k_rho 
        self.k_beta = -0.3      # k_beta < 0

        # 5. subscribe to Odemtry
        # to get the current position and orientation of robot

        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # 6. create a Twist publisher /cmd_vel
        # for the velocity commends
        # store that as object variable cmd_pub

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        # 7. set the publishing rate to 10 Hz

        self.rate = rospy.Rate(10)


    # helper function for wrapping of angle

    def wrap_angle(self, angle):

        #   while angle is greater than pi, subtract 2*pi
        #   while angle is <= -pi, add 2*pi

        while angle > pi:
            angle -= 2 * pi
        while angle <= -pi:
            angle += 2 * pi
        return angle


    # callback function for the odometry
    # to extract the robot position and orientation from 
    # Odometry measurements

    def odom_callback(self, msg):

        # 1. extract the x and the y positions
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # 2. get the orientaion quaternion and convert to euler angles

        orientation_quaternion = msg.pose.pose.orientation
        quaternion = [orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w]

        # 3. convert to euler angle and store the yaw only

        _, _, self.yaw = euler_from_quaternion(quaternion)


    # function to computer the movement of the robot

    def compute_control(self):

        # difference in the position

        dx = self.x_goal - self.x
        dy = self.y_goal - self.y
        dyaw = self.yaw_goal - self.yaw

        # compute the rho by euclidian distance
        
        rho = sqrt((self.x_goal - self.x) ** 2 + (self.y_goal - self.y) ** 2)

        if rho < 0.01:
            return 0, 0

        # compute the orientation error, alpha using atan2

        alpha = self.wrap_angle(atan2(dy, dx) - self.yaw)

        # compute the final orientation error, beta

        beta = dyaw - alpha

        # compute linear velocity and angular velocity

        v = self.k_rho * rho
        w = self.k_alpha * alpha + self.k_beta * beta

        # reverse the direction if the goal is behind

        if (-pi < alpha <= -pi / 2) or (pi / 2 < alpha <= pi):
            v = -v                                  # negative velocity

        # return v, w

        return v, w
    

    # run function

    def run(self):
        while not rospy.is_shutdown():

            # take the linear velocity and angular velocity and assign them to variables

            linear_vel, angular_vel = self.compute_control()

            # create a tiwst object

            twist = Twist()

            # set the linear velocities

            twist.linear.x = linear_vel
            twist.linear.y = 0
            twist.linear.z = 0

            # set the angular velocities

            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = angular_vel

            # publich the twist

            self.cmd_pub.publish(twist)

            # sleep to maintain the loop rate

            self.rate.sleep()

if __name__ == '__main__':
    try:
        
        # create the controller object and run
        
        controller = TurtleBot3Controller()
        controller.run()
    
    except rospy.ROSInterruptException:
        pass