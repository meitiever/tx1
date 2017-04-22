#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#

import geometry_msgs.msg
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from imu_interface import Interface

def parser():
  rospy.init_node('str2imu', anonymous=True)
  rate = rospy.Rate(10) # 1hz
  dc = rospy.Publisher('IMU2', Imu, queue_size=10)
  interface = Interface()

  while not rospy.is_shutdown():
    imu = interface.get_imu()
    if imu is not None:
      dc.publish(imu)
    rate.sleep()

  rospy.spin()

if __name__ == '__main__':
    parser()
