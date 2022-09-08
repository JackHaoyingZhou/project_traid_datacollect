#!/usr/bin/env python3

# Author: Anton Deguet
# Date: 2015-02-22

# (C) Copyright 2015-2021 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun dvrk_python dvrk_arm_test.py <arm-name>

import dvrk
import math
import sys
import time
import rospy
import numpy
import PyKDL
import argparse
import csv
import os
from joint_pos_recorder import JointPosRecorder

jpRecorder = JointPosRecorder(save_path='./data/PSM2_js', record_size=50)

# print with node id
def print_id(message):
    print('%s -> %s' % (rospy.get_caller_id(), message))

# example of application using arm.py
class example_application:

    # configuration
    def configure(self, robot_name, expected_interval, insertion):
        print_id('configuring dvrk_arm_test for %s' % robot_name)
        self.expected_interval = expected_interval
        self.insertion = insertion
        file = open('testdata1.csv', 'w+')
        self.write = csv.writer(file)
        header = ['measured global joint position', 'measured global cartesian position',
                  'measured RCM cartesian position', 'desired global joint position',
                  'desired global cartesian position', 'desired RCM cartesian position']
        self.write.writerow(header)
        self.arm = dvrk.arm(arm_name = robot_name,
                            expected_interval = expected_interval)

    # homing example-0.524
    #35 degree -- 0.611
    def home(self):
        print_id('starting enable')
        if not self.arm.enable(10):
            sys.exit('failed to enable within 10 seconds')
        print_id('starting home')
        if not self.arm.home(10):
            sys.exit('failed to home within 10 seconds')
        # get current joints just to set size
        print_id('move to starting position')
        goal = numpy.copy(self.arm.setpoint_jp())
        # print(type(goal))
        goal[0] = -0.10
        goal[1] = -0.611
        # goal[2] = 0.2
        # go to zero position, for PSM and ECM make sure 3rd joint is past cannula
        goal.fill(0)
        print(len(goal))
        
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2')
            or (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            goal[2] = 0.2 + self.insertion
        goal[1] = -0.611
        # move and wait
        print_id('moving to starting position')
        self.arm.move_jp(goal).wait()
        # try to move again to make sure waiting is working fine, i.e. not blocking
        print_id('testing move to current position')
        move_handle = self.arm.move_jp(goal)
        time.sleep(1.0) # add some artificial latency on this side
        move_handle.wait()
        print_id('home complete')

    # get methods
    def run_get(self, t, init=False):

        d1 = self.arm.measured_jp(extra = True)
        # d2 = self.arm.measured_cp()
        # d3 = self.arm.local.measured_cp()
        self.d4 = self.arm.setpoint_jp()
        # d5 = self.arm.setpoint_cp()
        # d6 = self.arm.local.setpoint_cp()
        if init:
            t = 0
        else:
            t = t + d1[1] - self.timestamp
        self.timestamp = d1[1]
        temp = [d1[0][0], self.d4[0],t]
        print(d1)
        print('feed',temp)
        self.write.writerow(temp)
        return t

    # goal joint control example
    def run_move_jp(self):
        print_id('starting move_jp')
        # get current position
        initial_joint_position = numpy.copy(self.arm.setpoint_jp())
        initial_joint_position[0] = 0
        initial_joint_position[1] = -0.611
        print_id('testing goal joint position for 2 joints out of %i' % initial_joint_position.size)
        amplitude = math.radians(1)
        # create a new goal starting with current position
        goal = numpy.copy(initial_joint_position)

        # first motion method
        goal[0] = math.radians(20.0)
        goal[1] = -0.611 # math.radians(-17.0)

        self.arm.move_jp(goal).wait()
        t = 0
        t = self.run_get(t, init=True)
        count = 0
        input("Press Enter to Start...")
        while True:
            if count == 5:
                count = 0
                input("Press Enter to continue...")
            input("Press Enter to continue")
            time.sleep(0.2)
            goal[0] = goal[0] - amplitude
            self.arm.servo_jp(goal)
            self.write.writerow(goal)
            jpRecorder.record(list(goal))
            print('saving')
            # t = self.run_get(t)
            #count += 1
            if goal[0] < math.radians(-20.0):
                input("Press Enter to EXIT...")
                self.arm.move_jp(initial_joint_position).wait()
                print_id('move_jp complete')
                # nput("Press Enter")
                break

    # utility to position tool/camera deep enough before cartesian examples
    def prepare_cartesian(self):
        # make sure the camera is past the cannula and tool vertical
        goal = numpy.copy(self.arm.setpoint_jp())
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2')
            or (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            # set in position joint mode
            goal[0] = -0.10
            goal[1] = -0.611 ##############################
            goal[2] = 0.2 + self.insertion
            goal[3] = 0.0
            self.arm.move_jp(goal).wait()

    # direct cartesian control example
    def run_move_cp(self):
        print_id('starting move_cp')
        self.prepare_cartesian()

        # create a new goal starting with current position
        initial_cartesian_position = PyKDL.Frame()
        initial_cartesian_position.p = self.arm.setpoint_cp().p
        initial_cartesian_position.M = self.arm.setpoint_cp().M
        goal = PyKDL.Frame()
        goal.p = self.arm.setpoint_cp().p
        goal.M = self.arm.setpoint_cp().M

        # motion parameters
        amplitude = math.radians(1) # 1 degree

        goal.p[0] = -0.0200
        self.arm.move_cp(goal).wait()
        self.run_get()
        theta0 = self.d4[0]
        theta = self.d4[0]
        while True:
            # input("Press Enter to continue...")
            print('joint',theta,theta+amplitude,math.tan(theta)*0.08,math.tan(theta+amplitude)*0.08)
            goal.p[0] = math.tan(theta+amplitude) * (0.2+self.insertion)
            self.arm.move_cp(goal).wait()
            self.run_get()
            theta = self.d4[0]
            if theta > -theta0:
                # back to initial position
                input("Press Enter to continue...")
                self.arm.move_cp(initial_cartesian_position).wait()
                self.run_get()
                print_id('move_cp complete')
                break

    # main method
    def run(self):
        try:
            self.home()
            self.run_move_jp()
            #self.run_move_cp()
        except KeyboardInterrupt:
            sys.exit()

if __name__ == '__main__':
    # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
    rospy.init_node('dvrk_arm_test', anonymous=True)
    # strip ros arguments
    argv = rospy.myargv(argv=sys.argv)

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        choices=['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    parser.add_argument('-s', '--insertion', type=float, default=0.0,
                        help = 'expected extra initialized insertion for probe by meters (real insertion - 0.2)')
    args = parser.parse_args(argv[1:]) # skip argv[0], script name

    application = example_application()
    application.configure(args.arm, args.interval, args.insertion)
    application.run()
    jpRecorder.flush()
