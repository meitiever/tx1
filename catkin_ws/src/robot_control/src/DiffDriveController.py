#!/usr/bin/python

import numpy as np
import math
import rospy

class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, max_speed, max_omega):
        # TODO for Student: Specify these parameters
        self.kp= 0.8
        self.ka= 1
        self.kb=-0 # i do not care angle
        self.MAX_SPEED = max_speed
        self.MAX_OMEGA = max_omega
        
    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should 
            be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot
            (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or
            is close enough
        """
        # YOUR CODE HERE
        dx=goal[0][0]-state[0][0]
        dy=goal[1][0]-state[1][0]
        rho=np.sqrt(dx**2+dy**2)
        #alpha=math.atan(dy/dx)-state[2][0]
        alpha=math.atan(dy/dx)
        #beta=-state[2][0]-alpha
        beta=-state[2][0]

        rabMatrix=np.array([[rho[0,0]],[alpha],[beta]])
        #rabMatrix=np.array([[rho[0,0]],[alpha[0,0]],[beta[0,0]]])
        SMatrix=np.array([[self.kp,0,0],[0,self.ka,self.kb]])
        OutputMatrix=np.dot(SMatrix,rabMatrix)

        v=OutputMatrix[0][0]
        omega=OutputMatrix[1][0]

        if v>=self.MAX_SPEED:
            v=self.MAX_SPEED

        if omega>=self.MAX_OMEGA:
            omega=self.MAX_OMEGA

        #rospy.loginfo('original v and omage is: [%f %f]', v, omega)

        sign=0
        if omega<0:
            sign=1
            omega=-omega

        #rospy.loginfo('value of v and omage is: [%f %f]', v, omega)

        if omega<0.2:
            omega=1.3
        elif omega<0.3:
            omega=1.4
        elif omega<0.5:
            omega=1.5
        elif omega<0.8:
            omega=1.6
        elif omega<2:
            omega=1.8

        if v<0.5:
            v=0.5
        elif v<1.5:
            v=0.6
        elif v<20:
            v=0.7

        if sign==1:
            omega=-omega

        done=False
        rospy.loginfo('distance between goal and pos is : [%f]', rho)
        if rho<=0.2:
            done=True

        return (v, omega, done)


        #pass
