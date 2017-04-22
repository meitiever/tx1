#!/usr/bin/env python
"""
ROS based interface for the Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2015.
"""

import roslib
import rospy
import sys

# Extra utility functions
from utility import *

from geometry_msgs.msg import (
    Twist,
)

def command_twist(twist):
    """
    Commands the robot to move with linear velocity vx and angular
    velocity wz
    """
    rospy.loginfo('a new message sender' + twist.)

def main(args):
    rospy.init_node('roboclaw_sender', anonymous=True)
    # Intialize the RobotControl object
    rospy.Subscriber("/cmd_vel", Twist, command_twist)
    rospy.loginfo('this is message sender')
    rospy.spin()

if __name__ == "__main__":
    main(sys.argv)
