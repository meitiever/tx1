#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""
# ROS imports
import roslib
import rospy

from std_msgs.msg import (
    Header,
    String,
)

import geometry_msgs.msg
import tf
import math
from sensor_msgs.msg import Imu


class Interface(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self):
        """
        Initialize the class
        """
        # Internal variables
        self._imu_received = False
        #self._vel_received = False
        #self._fusion_received = False
        #self._fusion = None
        #self._vel = None
        self._imu = None

        # ROS publishers and subscribers        
        #rospy.Subscriber("/robot/VEL", String, self._vel_callback)
        rospy.Subscriber("IMU", String, self._imu_callback)
        #rospy.Subscriber("/robot/VEL", String, self._vel_callback)

    def _imu_callback(self, data):
        """
        Callback function for IMU measurements
        """
        r = data.data.split(' ')
        self._imu = Imu()
        self._imu.orientation_covariance = [-1, 0, 0, -1, 0, 0, -1, 0, 0]
        self._imu.angular_velocity_covariance = [0, 0 , 0, 0 , 0, 0, 0 , 0 , 0]
        self._imu.linear_acceleration_covariance = [0 , 0 , 0, 0 , 0, 0, 0 , 0 , 0]
        self._imu.linear_acceleration.x = float(r[0])
        self._imu.linear_acceleration.y = float(r[1])
        self._imu.linear_acceleration.z = float(r[2])
        self._imu.angular_velocity.x = float(r[3])
        self._imu.angular_velocity.y = float(r[4])
        self._imu.angular_velocity.z = float(r[5])
        self._imu.orientation.x = float(r[7])
        self._imu.orientation.y = float(r[8])
        self._imu.orientation.z = float(r[6])
        self._imu.orientation.w = 0
        self._imu.header.seq = 0
        self._imu.header.frame_id = "base_link"
        self._imu.header.stamp = rospy.Time.now()
        self._imu_received = True

    def get_imu(self):
        if self._imu_received == False:
            return None
        else:
            return self._imu

    def _fusion_callback(self, fusion):
        """
        Callback function for IMU measurements
        """
        self._fusion = np.matrix([[]]).T
        self._no_fusion = False

    def get_fusion(self):
        if self._no_fusion == False:
            return None
        else:
            return self._fusion
        
