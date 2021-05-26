#!/usr/bin/env python
from map import *
from robot_control import RobotControl
import rospy
import time
import math
import sys
import os
import rospkg
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

        data_dict = {(0.18886572122573853, -1000.0, 0.29489263892173767): 'L',
        (0.19246011972427368, 0.27303528785705566, 0.4725402295589447): 'T',
        (-1000.0, -1000.0, 0.1216597780585289): 'F',
        (-1000.0, -1000.0, 0.5008593797683716): 'F',
        (0.4662621319293976, -1000.0, 0.2998603284358978): 'F',
        (0.11432650685310364, -1000.0, 0.49349287152290344): 'L',
        (-1000.0, 0.16683532297611237, -1000.0): 'F',
        (-1000.0, 0.4849688410758972, 0.1722767949104309): 'F',
        (0.36776286363601685, 0.48489582538604736, -1000.0): 'L',
        (0.11320717632770538, -1000.0, -1000.0): 'L',
        (0.4656304121017456, -1000.0, -1000.0): 'F',
        (0.36713093519210815, 0.4793507158756256, -1000.0): 'R',
        (0.19279010593891144, 0.20319049060344696, -1000.0): 'R',
        (0.1432543843984604, 0.20479263365268707, -1000.0): 'R',
        (0.23664286732673645, 0.40605753660202026, -1000.0): 'R',
        (-1000.0, -1000.0, 0.4744504690170288): 'F',
        (-1000.0, 0.4902779757976532, -1000.0): 'F',
        (0.16335883736610413, -1000.0, 0.4415828585624695): 'L',
        (-1000.0, 0.17930111289024353, 0.4783061742782593): 'F',
        (0.13843156397342682, 0.2047196328639984, -1000.0): 'R',
        (-1000.0, -1000.0, 0.22309862077236176): 'F',
        (-1000.0, 0.2079465538263321, 0.44593536853790283): 'F',
        (-1000.0, 0.3476245701313019, -1000.0): 'F',
        (0.19611884653568268, -1000.0, 0.17517587542533875): 'R',
        (0.47354668378829956, 0.4263686537742615, -1000.0): 'F',
        (-1000.0, 0.49951663613319397, 0.352539598941803): 'F',
        (0.22046276926994324, 0.49880141019821167, 0.2516286075115204): 'T',
        (-1000.0, -1000.0, 0.17484889924526215): 'F',
        (0.24679458141326904, 0.37764638662338257, 0.373322069644928): 'T',
        (0.1179201677441597, 0.16323581337928772, -1000.0): 'R',
        (0.11907172203063965, -1000.0, 0.4719002842903137): 'L',
        (-1000.0, 0.43713298439979553, 0.2100927233695984): 'F',
        (0.4946841597557068, -1000.0, -1000.0): 'F',
        (0.1934836506843567, 0.4065283238887787, -1000.0): 'R',
        (0.14272016286849976, 0.1979454755783081, -1000.0): 'R',
        (-1000.0, -1000.0, -1000.0): 'F',
        (0.4886481761932373, -1000.0, 0.4722394645214081): 'F',
        (0.16812939941883087, 0.45914918184280396, -1000.0): 'R',
        (0.11304689943790436, -1000.0, 0.2653895616531372): 'L',
        (0.24596309661865234, 0.2498926967382431, 0.5007567405700684): 'T',
        (0.49938568472862244, 0.25020188093185425, 0.5038331151008606): 'F',
        (0.16292355954647064, -1000.0, -1000.0): 'R',
        (0.2389470338821411, -1000.0, 0.447404146194458): 'L',
        (-1000.0, 0.30519160628318787, 0.3537727892398834): 'F',
        (0.4454415738582611, -1000.0, 0.1482175886631012): 'F',
        (0.4690057337284088, 0.48210495710372925, -1000.0): 'F',
        (0.16980811953544617, 0.3732264041900635, 0.37402617931365967): 'T',
        (0.11840330064296722, 0.22482730448246002, -1000.0): 'R',
        (0.19094285368919373, -1000.0, 0.37234944105148315): 'L',
        (0.13888894021511078, 0.4835090935230255, -1000.0): 'R',
        (0.2938598096370697, 0.17154090106487274, -1000.0): 'R',
        (0.45002952218055725, 0.49831652641296387, 0.24978582561016083): 'F',
        (-1000.0, -1000.0, 0.3537442684173584): 'F',
        (0.23689614236354828, -1000.0, -1000.0): 'R',
        (0.16832999885082245, -1000.0, 0.22385288774967194): 'L',
        (-1000.0, 0.17603366076946259, 0.4859517812728882): 'F',
        (0.46752336621284485, 0.18278983235359192, -1000.0): 'F',
        (0.26373186707496643, -1000.0, 0.46979808807373047): 'L',
        (0.269019216299057, 0.48895263671875, -1000.0): 'R',
        (0.11160784959793091, 0.4868141710758209, -1000.0): 'R'}
        
        while not rospy.is_shutdown():
            dir = input("which direction?")
            left_dist = self.getSensorValue('left')
            right_dist = self.getSensorValue('right')
            front_dist = self.getSensorValue('front')

            data_dict[(front_dist, left_dist, right_dist)] = dir
            print(data_dict)

            # self.hold_neutral() #remove if not necessary
            # ---- add your code for a particular behavior here ---

            # left sensor dist from center body 0.18 m
            # self.walk_forward()
            # self.turnleft()
            # self.turnright()
            # self.reactive_control()
            # change the sleep time to whatever is the appropriate control rate for simulation
            time.sleep(0.1)
    def distance(self, point1, point2){
        x = (point1[0]-point2[0])**2
        y = (point1[1]-point2[1])**2
        z = (point1[2]-point2[2])**2
        d = sqrt(x+y+z)
    }

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
