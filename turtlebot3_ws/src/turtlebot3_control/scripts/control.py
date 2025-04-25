#!/usr/bin/env/python3

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

import numpy as np
import math


class TurtleBot3Controller:
    def __init__(self):

        # 1. initialize the ROS node

        rospy.init_node('my_turtlebot3_control')
        rospy.loginfo("Turtlebot 3 Sigwart et. al. Controller Started...")

        # 2. set the initial position and orientation

        self.x = 0
        self.y = 0
        self.th = 0

        #3. set the goal of the robot

        self.x_goal = 2.0
        self.y_goal = 2.0

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


    # callback function for the odometry

    def odom_callback(self, msg):
