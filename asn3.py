#!/usr/bin/env python
from map import *
import rospy
import time
import math
import sys
import os
import rospkg
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Range, JointState
import tf2_ros
from tf2_msgs.msg import TFMessage
import geometry_msgs.msg
import time
import numpy as np
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from me_cs301_robots.cfg import HexapodJointControlConfig, RWJointControlConfig
import math


class RobotControl(object):
    def __init__(self, robot_type='rollerwalker'):
        self.robot_type = robot_type
        if self.robot_type == 'hexapod':
            rospy.init_node('hexapod_control', anonymous=True)
        elif self.robot_type == 'rollerwalker':
            rospy.init_node('roller_walker_control', anonymous=True)
            
        print('INIT NODE')
        
        self.initialize_publishers()
        self.initialize_subscribers()
        self.sim_time = 0.0
        
        self.robot_pose = Pose()
        self.front_sensor_val = Float32()
        self.left_sensor_val = Float32()
        self.right_sensor_val = Float32()

        self.joint_states = JointState()
        self.SENSOR_TYPES = ['front', 'left', 'right']
        # self.is_go_down = False

        self.initialize_publisher_msgs()

        if self.robot_type == 'hexapod':
            self.paradigm_reconfigure_srv = DynamicReconfigureServer(HexapodJointControlConfig, self.reconfig_paradigm_cb)
        else:
            self.paradigm_reconfigure_srv = DynamicReconfigureServer(RWJointControlConfig, self.reconfig_paradigm_cb)

    def degToRad(self, deg):
        return deg*math.pi/180.0
        
    def initialize_publisher_msgs(self):
        if self.robot_type == 'hexapod':
            self.leg1_j1 = Float32()
            self.leg1_j2 = Float32()
            self.leg1_j3 = Float32()

            self.leg2_j1 = Float32()
            self.leg2_j2 = Float32()
            self.leg2_j3 = Float32()
            
            self.leg3_j1 = Float32()
            self.leg3_j2 = Float32()
            self.leg3_j3 = Float32()

            self.leg4_j1 = Float32()
            self.leg4_j2 = Float32()
            self.leg4_j3 = Float32()
            
            self.leg5_j1 = Float32()
            self.leg5_j2 = Float32()
            self.leg5_j3 = Float32()
            
            self.leg6_j1 = Float32()
            self.leg6_j2 = Float32()
            self.leg6_j3 = Float32()

        elif self.robot_type == 'rollerwalker':
            self.leg1_j1 = Float32()
            self.leg1_j2 = Float32()
            self.leg1_j3 = Float32()

            self.leg2_j1 = Float32()
            self.leg2_j2 = Float32()
            self.leg2_j3 = Float32()

            self.leg3_j1 = Float32()
            self.leg3_j2 = Float32()
            self.leg3_j3 = Float32()

            self.leg4_j1 = Float32()
            self.leg4_j2 = Float32()
            self.leg4_j3 = Float32()

    def initialize_publishers(self):
        # initialized latched publishers
        if self.robot_type == 'hexapod':
            self.leg1_j1_pub = rospy.Publisher('/leg1_j1', Float32, queue_size=1, latch=True)
            self.leg1_j2_pub = rospy.Publisher('/leg1_j2', Float32, queue_size=1, latch=True)
            self.leg1_j3_pub = rospy.Publisher('/leg1_j3', Float32, queue_size=1, latch=True)
            
            self.leg2_j1_pub = rospy.Publisher('/leg2_j1', Float32, queue_size=1, latch=True)
            self.leg2_j2_pub = rospy.Publisher('/leg2_j2', Float32, queue_size=1, latch=True)
            self.leg2_j3_pub = rospy.Publisher('/leg2_j3', Float32, queue_size=1, latch=True)

            self.leg3_j1_pub = rospy.Publisher('/leg3_j1', Float32, queue_size=1, latch=True)
            self.leg3_j2_pub = rospy.Publisher('/leg3_j2', Float32, queue_size=1, latch=True)
            self.leg3_j3_pub = rospy.Publisher('/leg3_j3', Float32, queue_size=1, latch=True)

            self.leg4_j1_pub = rospy.Publisher('/leg4_j1', Float32, queue_size=1, latch=True)
            self.leg4_j2_pub = rospy.Publisher('/leg4_j2', Float32, queue_size=1, latch=True)
            self.leg4_j3_pub = rospy.Publisher('/leg4_j3', Float32, queue_size=1, latch=True)

            self.leg5_j1_pub = rospy.Publisher('/leg5_j1', Float32, queue_size=1, latch=True)
            self.leg5_j2_pub = rospy.Publisher('/leg5_j2', Float32, queue_size=1, latch=True)
            self.leg5_j3_pub = rospy.Publisher('/leg5_j3', Float32, queue_size=1, latch=True)

            self.leg6_j1_pub = rospy.Publisher('/leg6_j1', Float32, queue_size=1, latch=True)
            self.leg6_j2_pub = rospy.Publisher('/leg6_j2', Float32, queue_size=1, latch=True)
            self.leg6_j3_pub = rospy.Publisher('/leg6_j3', Float32, queue_size=1, latch=True)

        elif self.robot_type == 'rollerwalker':
            self.leg1_j1_pub = rospy.Publisher('/leg1_j1', Float32, queue_size=1, latch=True)
            self.leg1_j2_pub = rospy.Publisher('/leg1_j2', Float32, queue_size=1, latch=True)
            self.leg1_j3_pub = rospy.Publisher('/leg1_j3', Float32, queue_size=1, latch=True)
            
            self.leg2_j1_pub = rospy.Publisher('/leg2_j1', Float32, queue_size=1, latch=True)
            self.leg2_j2_pub = rospy.Publisher('/leg2_j2', Float32, queue_size=1, latch=True)
            self.leg2_j3_pub = rospy.Publisher('/leg2_j3', Float32, queue_size=1, latch=True)

            self.leg3_j1_pub = rospy.Publisher('/leg3_j1', Float32, queue_size=1, latch=True)
            self.leg3_j2_pub = rospy.Publisher('/leg3_j2', Float32, queue_size=1, latch=True)
            self.leg3_j3_pub = rospy.Publisher('/leg3_j3', Float32, queue_size=1, latch=True)

            self.leg4_j1_pub = rospy.Publisher('/leg4_j1', Float32, queue_size=1, latch=True)
            self.leg4_j2_pub = rospy.Publisher('/leg4_j2', Float32, queue_size=1, latch=True)
            self.leg4_j3_pub = rospy.Publisher('/leg4_j3', Float32, queue_size=1, latch=True)
    
    def initialize_subscribers(self):
        rospy.Subscriber('/simTime', Float32, self.simTime_cb)
        rospy.Subscriber('/tf', TFMessage, self.tf_cb)

        rospy.Subscriber('/frontSensorDistance', Range, self.distance_front_cb)
        rospy.Subscriber('/leftSensorDistance',  Range, self.distance_left_cb)
        rospy.Subscriber('/rightSensorDistance', Range, self.distance_right_cb)

        if self.robot_type == 'hexapod':
            rospy.Subscriber('/hexapod/joint_states', JointState, self.joint_states_cb)
        elif self.robot_type == 'rollerwalker':
            rospy.Subscriber('/rollerwalker/joint_states', JointState, self.joint_states_cb)
    
    # subscriber callbacks
    def simTime_cb(self, msg):
        self.sim_time = msg.data
    
    def tf_cb(self, msg):
        self.robot_pose.position.x = msg.transforms[0].transform.translation.x
        self.robot_pose.position.y = msg.transforms[0].transform.translation.y
        self.robot_pose.position.z = msg.transforms[0].transform.translation.z
        self.robot_pose.orientation.x = msg.transforms[0].transform.rotation.x
        self.robot_pose.orientation.y = msg.transforms[0].transform.rotation.y
        self.robot_pose.orientation.z = msg.transforms[0].transform.rotation.z
        self.robot_pose.orientation.w = msg.transforms[0].transform.rotation.w

    #Sensor callbacks Same for both robots. 
    def distance_front_cb(self, msg):
        self.front_sensor_val.data = msg.range
    
    def distance_left_cb(self, msg):
        self.left_sensor_val.data = msg.range    

    def distance_right_cb(self, msg):
        self.right_sensor_val.data = msg.range
        
    def joint_states_cb(self, msg):
        self.joint_states = msg

    def publish_joint_values(self): 
        if self.robot_type == 'hexapod':
            self.leg1_j1_pub.publish(self.leg1_j1)
            self.leg1_j2_pub.publish(self.leg1_j2)
            self.leg1_j3_pub.publish(self.leg1_j3)

            self.leg2_j1_pub.publish(self.leg2_j1)
            self.leg2_j2_pub.publish(self.leg2_j2)
            self.leg2_j3_pub.publish(self.leg2_j3)
            
            self.leg3_j1_pub.publish(self.leg3_j1)
            self.leg3_j2_pub.publish(self.leg3_j2)
            self.leg3_j3_pub.publish(self.leg3_j3)

            self.leg4_j1_pub.publish(self.leg4_j1)
            self.leg4_j2_pub.publish(self.leg4_j2)
            self.leg4_j3_pub.publish(self.leg4_j3)

            self.leg5_j1_pub.publish(self.leg5_j1)
            self.leg5_j2_pub.publish(self.leg5_j2)
            self.leg5_j3_pub.publish(self.leg5_j3)

            self.leg6_j1_pub.publish(self.leg6_j1)
            self.leg6_j2_pub.publish(self.leg6_j2)
            self.leg6_j3_pub.publish(self.leg6_j3)

        elif self.robot_type == 'rollerwalker':
            self.leg1_j1_pub.publish(self.leg1_j1)
            self.leg1_j2_pub.publish(self.leg1_j2)
            self.leg1_j3_pub.publish(self.leg1_j3)

            self.leg2_j1_pub.publish(self.leg2_j1)
            self.leg2_j2_pub.publish(self.leg2_j2)
            self.leg2_j3_pub.publish(self.leg2_j3)
            
            self.leg3_j1_pub.publish(self.leg3_j1)
            self.leg3_j2_pub.publish(self.leg3_j2)
            self.leg3_j3_pub.publish(self.leg3_j3)

            self.leg4_j1_pub.publish(self.leg4_j1)
            self.leg4_j2_pub.publish(self.leg4_j2)
            self.leg4_j3_pub.publish(self.leg4_j3)
    
    def reconfig_paradigm_cb(self, config, level):
        if self.robot_type == 'hexapod':
            self.leg1_j1.data = config.leg1_j1
            self.leg1_j2.data = config.leg1_j2
            self.leg1_j3.data = config.leg1_j3

            self.leg2_j1.data = config.leg2_j1
            self.leg2_j2.data = config.leg2_j2
            self.leg2_j3.data = config.leg2_j3

            self.leg3_j1.data = config.leg3_j1
            self.leg3_j2.data = config.leg3_j2
            self.leg3_j3.data = config.leg3_j3

            self.leg4_j1.data = config.leg4_j1
            self.leg4_j2.data = config.leg4_j2
            self.leg4_j3.data = config.leg4_j3

            self.leg5_j1.data = config.leg5_j1
            self.leg5_j2.data = config.leg5_j2
            self.leg5_j3.data = config.leg5_j3

            self.leg6_j1.data = config.leg6_j1
            self.leg6_j2.data = config.leg6_j2
            self.leg6_j3.data = config.leg6_j3
            
        elif self.robot_type == 'rollerwalker':
            self.leg1_j1.data = config.leg1_j1
            self.leg1_j2.data = config.leg1_j2
            self.leg1_j3.data = config.leg1_j3

            self.leg2_j1.data = config.leg2_j1
            self.leg2_j2.data = config.leg2_j2
            self.leg2_j3.data = config.leg2_j3

            self.leg3_j1.data = config.leg3_j1
            self.leg3_j2.data = config.leg3_j2
            self.leg3_j3.data = config.leg3_j3

            self.leg4_j1.data = config.leg4_j1
            self.leg4_j2.data = config.leg4_j2
            self.leg4_j3.data = config.leg4_j3


        self.publish_joint_values()

        return config
    
    #getters 
    def getSensorValue(self, sensor_type='front'):
        assert sensor_type in self.SENSOR_TYPES
        if sensor_type=='front':
            return self.front_sensor_val.data
        elif sensor_type == 'left':
            return self.left_sensor_val.data
        elif sensor_type == 'right':
            return self.right_sensor_val.data
    
    def getMotorCurrentJointPosition(self, motor_id_string='leg1_j1'):
        assert motor_id_string in self.joint_states.name
        motor_id = self.joint_states.name.index(motor_id_string)
        return self.joint_states.position[motor_id]
    
    def getRobotWorldLocation(self):
        return self.robot_pose.position, self.robot_pose.orientation
    
    def getCurrentSimTime(self):
        return self.sim_time
    
    #setters
    
    def setMotorTargetJointPosition(self, motor_id_string='leg1_j1', target_joint_angle=0.0):
        if motor_id_string == 'leg1_j1':
            self.leg1_j1.data = target_joint_angle
        elif motor_id_string == 'leg1_j2':
            self.leg1_j2.data = target_joint_angle
        elif motor_id_string == 'leg1_j3':
            self.leg1_j3.data = target_joint_angle
        elif motor_id_string == 'leg2_j1':
            self.leg2_j1.data = target_joint_angle
        elif motor_id_string == 'leg2_j2':
            self.leg2_j2.data = target_joint_angle
        elif motor_id_string == 'leg2_j3':
            self.leg2_j3.data = target_joint_angle
        elif motor_id_string == 'leg3_j1':
            self.leg3_j1.data = target_joint_angle
        elif motor_id_string == 'leg3_j2':
            self.leg3_j2.data = target_joint_angle
        elif motor_id_string == 'leg3_j3':
            self.leg3_j3.data = target_joint_angle
        elif motor_id_string == 'leg4_j1':
            self.leg4_j1.data = target_joint_angle
        elif motor_id_string == 'leg4_j2':
            self.leg4_j2.data = target_joint_angle
        elif motor_id_string == 'leg4_j3':
            self.leg4_j3.data = target_joint_angle
        elif motor_id_string == 'leg5_j1':
            self.leg5_j1.data = target_joint_angle
        elif motor_id_string == 'leg5_j2':
            self.leg5_j2.data = target_joint_angle
        elif motor_id_string == 'leg5_j3':
            self.leg5_j3.data = target_joint_angle
        elif motor_id_string == 'leg6_j1':
            self.leg6_j1.data = target_joint_angle
        elif motor_id_string == 'leg6_j2':
            self.leg6_j2.data = target_joint_angle
        elif motor_id_string == 'leg6_j3':
            self.leg6_j3.data = target_joint_angle
        
        self.publish_joint_values()

sys.path.append(os.path.join(
    rospkg.RosPack().get_path('me_cs301_robots'), 'scripts'))



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

        center_dict = {}
        data_dict = {}
        
        while not rospy.is_shutdown():
            dir = input("which direction?")
            left_dist = self.getSensorValue('left')
            right_dist = self.getSensorValue('right')
            front_dist = self.getSensorValue('front')

            center_dict[(front_dist, left_dist, right_dist)] = dir
            print(center_dict)

            # self.hold_neutral() #remove if not necessary
            # ---- add your code for a particular behavior here ---

            # left sensor dist from center body 0.18 m
            # self.walk_forward()
            # self.turnleft()
            # self.turnright()
            # self.reactive_control()
            # change the sleep time to whatever is the appropriate control rate for simulation
            time.sleep(0.1)
    
    def distance(self, point1, point2):
        x = (point1[0]-point2[0])**2
        y = (point1[1]-point2[1])**2
        z = (point1[2]-point2[2])**2
        d = sqrt(x+y+z)
    

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

    def reactive_control(self, sensor_data):
        left_dist = sensor_data[1] + 0.08
        right_dist = sensor_data[2] + 0.08
        front_dist = sensor_data[0] + 0.212
        if right_dist < 0:
            right_dist = 999
        if left_dist < 0:
            left_dist = 999
        if front_dist < 0:
            front_dist = 999

        if left_dist <= 0.4 and front_dist <= 0.4 and right_dist <= 0.4:
            return 'T'
        elif left_dist <= 0.4 and front_dist <= 0.4:
            return 'R'
        elif right_dist <= 0.4 and front_dist <= 0.4:
            return 'L'
        return 'F'

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
