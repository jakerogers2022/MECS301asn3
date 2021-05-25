#!/usr/bin/env python
import rospy
import time
import math
import sys
import os
import rospkg
sys.path.append(os.path.join(rospkg.RosPack().get_path('me_cs301_robots'), 'scripts'))
from robot_control import RobotControl
from map import *

'''
MotorIDstrings for Hexapod follow the convention legN_jM, where N can be 1,2,3,4,5,6 (for each of the 6 legs) and M can 1,2,3. M=1 is the joint closest to the body and M=3 is the joint farthest from the body.

Available Robot API commands:

1. setMotorTargetJointPosition(motor_id_string, target_joint_angle) - send the target position for motor_id_string to CoppeliaSim
2. getSensorValue(sensor_type), where sensor_type can be from ['front', 'left', 'right'] - retrieves the current reading of the sensor sensor_type
3. getMotorCurrentJointPosition(motor_id_string) - retrieves the current angle for motor motor_id_string
4. getRobotWorldLocation() - returns (position, orientation) of the robot with respect to the world frame. Note, that orientation is represented as a quaternion.
5. getCurrentSimTime() - returns the current simulation time

Helper functions

degToRad() - Converts degrees to radians

Note that, this list of API functions could potentially grow. You will be notified via Canvas if anything is updated

'''

class HexapodControl(RobotControl):
    def __init__(self):
        super(HexapodControl, self).__init__(robot_type='hexapod')
        rospy.loginfo("hexapod setup complete")
        self.hold_neutral()
        time.sleep(2.0)

        #main control loop
        while not rospy.is_shutdown(): 
            # self.hold_neutral() #remove if not necessary
            # ---- add your code for a particular behavior here ---

            #left sensor dist from center body 0.18 m
            #self.walk_forward()
            #self.turnleft()
            #self.turnright()
            #self.reactive_control()
            time.sleep(0.1) # change the sleep time to whatever is the appropriate control rate for simulation
        

    def adjust_path(self):
        left_dist = self.getSensorValue('left') + 0.08
        right_dist = self.getSensorValue('right') + 0.08
        front_dist = self.getSensorValue('front') + 0.08
        if (left_dist > 0.51 and left_dist < 0.7) or (right_dist < 0.4 and right_dist > 0):
            self.hold_neutral()
            self.adjust_left()
        if (right_dist > 0.51 and right_dist < 0.7) or (left_dist < 0.4 and left_dist > 0):
            self.hold_neutral()
            self.adjust_right()
    
    def adjust_left(self):
        self.setMotorTargetJointPosition('leg1_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg3_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg5_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(3))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(-3))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(3))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(-3))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(3))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(-3))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        time.sleep(0.5)
        self.setMotorTargetJointPosition('leg2_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg4_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg6_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(0))
        time.sleep(1)
        self.hold_neutral()
        time.sleep(0.5)

    def adjust_right(self):
        self.setMotorTargetJointPosition('leg1_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg3_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg5_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(-3))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(3))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(-3))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(3))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(-3))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(3))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        time.sleep(0.5)
        self.setMotorTargetJointPosition('leg2_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg4_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg6_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(0))
        time.sleep(1)
        self.hold_neutral()
        time.sleep(0.5)

    def reactive_control(self):
        left_dist = self.getSensorValue('left') + 0.08
        right_dist = self.getSensorValue('right') + 0.08
        front_dist = self.getSensorValue('front') + 0.212
        if right_dist < 0:
            right_dist = 999
        if left_dist < 0:
            left_dist = 999
        if front_dist < 0:
            front_dist = 999

        if left_dist <= 0.4 and front_dist <= 0.4 and right_dist <= 0.4:
            self.turnaround()
        elif left_dist <= 0.4 and front_dist <= 0.4:
            self.turnright()
        elif right_dist <= 0.4 and front_dist <= 0.4:
            self.turnleft()

    def turn_slightly_right(self):
        self.setMotorTargetJointPosition('leg1_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg3_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg5_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(-5))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(5))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(-5))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(5))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(-5))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(5))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        time.sleep(0.5)
        self.setMotorTargetJointPosition('leg2_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg4_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg6_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(0))
        time.sleep(1)
        self.hold_neutral()
        time.sleep(0.5)

    def turn_small_right(self):
        self.setMotorTargetJointPosition('leg1_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg3_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg5_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(-10))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(10))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(-10))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(10))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(-10))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(10))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        time.sleep(0.5)
        self.setMotorTargetJointPosition('leg2_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg4_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg6_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(0))
        time.sleep(1)
        self.hold_neutral()
        time.sleep(0.5)

    def turn_slightly_left(self):
        self.setMotorTargetJointPosition('leg1_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg3_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg5_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(5))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(-5))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(5))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(-5))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(5))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(-5))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        time.sleep(0.5)
        self.setMotorTargetJointPosition('leg2_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg4_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg6_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(0))
        time.sleep(1)
        self.hold_neutral()
        time.sleep(0.5)

    def turn_small_left(self):
        self.setMotorTargetJointPosition('leg1_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg3_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg5_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(10))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(-10))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(10))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(-10))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(10))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(-10))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        time.sleep(0.5)
        self.setMotorTargetJointPosition('leg2_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg4_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg6_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(0))
        time.sleep(1)
        self.hold_neutral()
        time.sleep(0.5)

    def walk_backward(self):
        print('joint 2 up')
        self.setMotorTargetJointPosition('leg2_j2', -.75)
        self.setMotorTargetJointPosition('leg6_j2', -.75)
        time.sleep(1)
        print('joint 1 forward')
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(-60))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(60))
        self.setMotorTargetJointPosition('leg2_j3', self.degToRad(60))
        self.setMotorTargetJointPosition('leg6_j3', self.degToRad(60))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j3', self.degToRad(60))
        time.sleep(1)
        print('joint 2 down')
        self.setMotorTargetJointPosition('leg2_j2', 1.5)
        self.setMotorTargetJointPosition('leg6_j2', 1.5)
        time.sleep(2)

    def walk_forward(self):
        self.setMotorTargetJointPosition('leg1_j3', self.degToRad(40))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(60))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(-60))

        self.setMotorTargetJointPosition('leg4_j2', -.75)
        time.sleep(0.5)
        self.setMotorTargetJointPosition('leg3_j3', self.degToRad(120))
        self.setMotorTargetJointPosition('leg3_j2', -0.4)
        self.setMotorTargetJointPosition('leg5_j3', self.degToRad(120))
        self.setMotorTargetJointPosition('leg5_j2', -0.4)

        self.setMotorTargetJointPosition('leg4_j3', self.degToRad(180))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg4_j2', 1.5)
        time.sleep(1)
        self.setMotorTargetJointPosition('leg4_j3', self.degToRad(60))
        self.setMotorTargetJointPosition('leg4_j2', -.75)
        self.setMotorTargetJointPosition('leg3_j2', -0.75)
        self.setMotorTargetJointPosition('leg5_j2', -0.75)
        time.sleep(1)

    def turnright(self):
        self.setMotorTargetJointPosition('leg1_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg3_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg5_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(-30))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(30))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(-30))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(30))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(-30))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(30))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        time.sleep(0.5)
        self.setMotorTargetJointPosition('leg2_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg4_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg6_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(30))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(-30))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(30))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(-30))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(30))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(-30))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg2_j2', -0.5)
        self.setMotorTargetJointPosition('leg4_j2', -0.5)
        self.setMotorTargetJointPosition('leg6_j2', -0.5)
        time.sleep(0.5)
        self.setMotorTargetJointPosition('leg1_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg3_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg5_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(-24))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(24))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(-24))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(24))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(-24))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(24))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        time.sleep(0.5)
        self.setMotorTargetJointPosition('leg2_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg4_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg6_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(0))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        self.setMotorTargetJointPosition('leg2_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg4_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg6_j2', self.degToRad(-90))
        time.sleep(1)
        self.hold_neutral()
        time.sleep(0.5)
    
    def turnleft(self):
        self.setMotorTargetJointPosition('leg1_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg3_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg5_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(30))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(-30))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(30))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(-30))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(30))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(-30))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        time.sleep(0.5)
        self.setMotorTargetJointPosition('leg2_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg4_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg6_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(-30))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(30))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(-30))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(30))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(-30))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(30))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg2_j2', -0.5)
        self.setMotorTargetJointPosition('leg4_j2', -0.5)
        self.setMotorTargetJointPosition('leg6_j2', -0.5)
        time.sleep(0.5)
        self.setMotorTargetJointPosition('leg1_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg3_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg5_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(24))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(-24))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(24))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(-24))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(24))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(-24))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        time.sleep(0.5)
        self.setMotorTargetJointPosition('leg2_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg4_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg6_j2', self.degToRad(-90))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg2_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg3_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg4_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg5_j1', self.degToRad(0))
        self.setMotorTargetJointPosition('leg6_j1', self.degToRad(0))
        time.sleep(1)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        self.setMotorTargetJointPosition('leg2_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg4_j2', self.degToRad(-90))
        self.setMotorTargetJointPosition('leg6_j2', self.degToRad(-90))
        time.sleep(1)
        self.hold_neutral()
        time.sleep(0.5)

    def turnaround(self):
        self.turnleft()
        self.turnleft()

    def hold_neutral(self):
        # --- simple example of a behavior ---- #
        self.setMotorTargetJointPosition('leg1_j1', 0.0)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg1_j3', 2.09)

        self.setMotorTargetJointPosition('leg2_j1', 0.0)
        self.setMotorTargetJointPosition('leg2_j2', -0.5)
        self.setMotorTargetJointPosition('leg2_j3', 2.09)

        self.setMotorTargetJointPosition('leg3_j1', 0.0)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j3', 2.09)

        self.setMotorTargetJointPosition('leg4_j1', 0.0)
        self.setMotorTargetJointPosition('leg4_j2', -0.5)
        self.setMotorTargetJointPosition('leg4_j3', 2.09)

        self.setMotorTargetJointPosition('leg5_j1', 0.0)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j3', 2.09)

        self.setMotorTargetJointPosition('leg6_j1', 0.0)
        self.setMotorTargetJointPosition('leg6_j2', -0.5)
        self.setMotorTargetJointPosition('leg6_j3', 2.09)


if __name__ == "__main__":
    q = HexapodControl()
    rospy.spin()