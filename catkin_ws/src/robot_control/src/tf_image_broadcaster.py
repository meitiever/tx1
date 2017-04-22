#!/usr/bin/env python  
import roslib
roslib.load_manifest('robot_control')
import rospy

import tf
import turtlesim.msg

if __name__ == '__main__':
    rospy.init_node('robot_image_broadcaster')
    #turtlename = rospy.get_param('~turtle')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.11, 0.0, 0.09),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "base_image",
                         "base_link")

        rate.sleep()

