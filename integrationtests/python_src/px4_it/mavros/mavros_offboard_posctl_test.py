#!/usr/bin/env python2
#***************************************************************************
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#***************************************************************************/

#
# @author Andreas Antener <andreas@uaventure.com>
#
# The shebang of this file is currently Python2 because some
# dependencies such as pymavlink don't play well with Python3 yet.
from __future__ import division

PKG = 'px4'

import subprocess
import rospy
import math
import numpy as np
import sys
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_test_common_uav0 import MavrosTestCommon as UAV0
from mavros_test_common_uav1 import MavrosTestCommon as UAV1
from mavros_test_common_uav2 import MavrosTestCommon as UAV2
from voronoi import Controller
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler
import os

from gazebo_msgs.msg import ModelStates

class MavrosOffboardPosctlTest():
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def __init__(self):

        self.pos0 = PoseStamped()
        self.radius0 = 1
        self.pos1 = PoseStamped()
        self.radius1 = 1
        self.pos2 = PoseStamped()
        self.radius2 = 1

        self.evader = ModelStates()
        self.uav0_state = ModelStates()
        self.uav1_state = ModelStates()
        self.uav2_state = ModelStates()

        self.uav0 = UAV0()
        self.uav1 = UAV1()
        self.uav2 = UAV2()

        self.voronoi_run_flag = False
        # ROS Subscrib
        self.gazebo_model_state_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_model_state_callback)

        # ROS Publish
        # uav0
        self.pos_setpoint_pub0 = rospy.Publisher('uav0/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        # uav1
        self.pos_setpoint_pub1 = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        # uav2
        self.pos_setpoint_pub2 = rospy.Publisher('uav2/mavros/setpoint_position/local', PoseStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

        # self.voronoi_thread = Thread(target=self.voronoi_run, args=())
        # self.voronoi_thread.daemon = True
        # self.voronoi_thread.start()
    #
    def gazebo_model_state_callback(self, data):
        for i in range(len(data.name)):
            # rospy.loginfo("data.name = {0}".format(data.name))
            if data.name[i] == "husky_alpha":
                self.evader.name = data.name[i]
                self.evader.pose = data.pose[i]
            if data.name[i] == "iris0":
                self.uav0_state.name = data.name[i]
                self.uav0_state.pose = data.pose[i]

            if data.name[i] == "iris1":
                self.uav1_state.name = data.name[i]
                self.uav1_state.pose = data.pose[i]
            if data.name[i] == "iris2":
                self.uav2_state.name = data.name[i]
                self.uav2_state.pose = data.pose[i]
    #
    def voronoi_run(self):
        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            if self.evader_captured(15) == 1 or self.evader_captured(15) == 2 or self.evader_captured(15) == 3:
                if not self.voronoi_run_flag:
                    self.voronoi_run_flag = True
                    # subprocess.Popen('python voronoi.py', shell=True, stdout=subprocess.PIPE)
                    os.system("python voronoi.py")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def send_pos(self):
        rate = rospy.Rate(10)  # Hz

        self.pos0.header = Header()
        self.pos0.header.frame_id = "base_footprint0"

        self.pos1.header = Header()
        self.pos1.header.frame_id = "base_footprint1"

        self.pos2.header = Header()
        self.pos2.header.frame_id = "base_footprint2"
        while not rospy.is_shutdown():
            self.pos0.header.stamp = rospy.Time.now()
            self.pos1.header.stamp = rospy.Time.now()
            self.pos2.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub0.publish(self.pos0)
            self.pos_setpoint_pub1.publish(self.pos1)
            self.pos_setpoint_pub2.publish(self.pos2)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, p0, p1, p2, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0}, y:{1}, z:{2}".format(
                self.uav0.local_position.pose.position, self.uav0.local_position.pose.
                position, self.uav0.local_position.pose.position))

        desired0 = np.array((p0[0], p0[1], p0[2]))
        desired1 = np.array((p1[0], p1[1], p1[2]))
        desired2 = np.array((p2[0], p2[1], p2[2]))
        pos0 = np.array((self.uav0.local_position.pose.position.x,
                        self.uav0.local_position.pose.position.y,
                        self.uav0.local_position.pose.position.z))
        pos1 = np.array((self.uav1.local_position.pose.position.x,
                        self.uav1.local_position.pose.position.y,
                        self.uav1.local_position.pose.position.z))
        pos2 = np.array((self.uav2.local_position.pose.position.x,
                        self.uav2.local_position.pose.position.y,
                        self.uav2.local_position.pose.position.z))
        return (np.linalg.norm(desired0 - pos0) < offset) and (np.linalg.norm(desired1 - pos1) < offset)and(np.linalg.norm(desired2 - pos2) < offset)

    def reach_position(self, p0, p1, p2, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos0.pose.position.x = p0[0]
        self.pos0.pose.position.y = p0[1]
        self.pos0.pose.position.z = p0[2]

        self.pos1.pose.position.x = p1[0]
        self.pos1.pose.position.y = p1[1]
        self.pos1.pose.position.z = p1[2]

        self.pos2.pose.position.x = p2[0]
        self.pos2.pose.position.y = p2[1]
        self.pos2.pose.position.z = p2[2]

        # rospy.loginfo("home_position = {0}".format(self.uav0.home_position.position))
        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos0.pose.orientation = Quaternion(*quaternion)
        self.pos1.pose.orientation = Quaternion(*quaternion)
        self.pos2.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        while True:
            if self.is_at_position(p0, p1, p2, self.radius1):
                rospy.loginfo("position reached | seconds: of {0}".format(
                   timeout))
                reached = True
                break

            if self.evader_captured(15) == 1:
                self.pos0.pose.position.x = self.evader.pose.position.x -75
                self.pos0.pose.position.y = self.evader.pose.position.y -27.5
                self.pos0.pose.position.z = 15
                if not self.voronoi_run_flag:
                    self.voronoi_run_flag = True
                    pid = subprocess.Popen([sys.executable, "voronoi.py"]) # Call subprocess


            elif self.evader_captured(15) == 2:
                self.pos1.pose.position.x = self.evader.pose.position.x -75
                self.pos1.pose.position.y = self.evader.pose.position.y -52.5
                self.pos1.pose.position.z = 15
                if not self.voronoi_run_flag:
                    self.voronoi_run_flag = True
                    pid = subprocess.Popen([sys.executable, "voronoi.py"]) # Call subprocess

            elif self.evader_captured(15) == 3:
                self.pos2.pose.position.x = self.evader.pose.position.x -75
                self.pos2.pose.position.y = self.evader.pose.position.y -77.5
                self.pos2.pose.position.z = 15
                if not self.voronoi_run_flag:
                    self.voronoi_run_flag = True
                    pid = subprocess.Popen([sys.executable, "voronoi.py"]) # Call subprocess


            rospy.loginfo(
                "attempting to reach position | p1: {0}, p2: {1}, p3: {2} | current position p0: {3}, p1: {4}, p2: {5}".
                format(self.pos0.pose.position, self.pos1.pose.position, self.pos2.pose.position, self.uav0.local_position.pose.position,
                   self.uav1.local_position.pose.position,
                   self.uav2.local_position.pose.position))


            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

    #
    def evader_captured(self, offset):
        pos0 = np.array((self.uav0_state.pose.position.x, self.uav0_state.pose.position.y))
        pos1 = np.array((self.uav1_state.pose.position.x, self.uav1_state.pose.position.y))
        pos2 = np.array((self.uav2_state.pose.position.x, self.uav2_state.pose.position.y))
        pos_evader = np.array((self.evader.pose.position.x, self.evader.pose.position.y))

        if np.linalg.norm(pos0 - pos_evader) < offset:
            rospy.loginfo("evader captured by uav0!!!")
            return 1
        elif np.linalg.norm(pos1 - pos_evader) < offset:
            rospy.loginfo("evader captured by uav1!!!")
            return 2
        elif np.linalg.norm(pos2 - pos_evader) < offset:
            rospy.loginfo("evader captured by uav2!!!")
            return 3
        else:
            return 0
    #
    def test_posctl(self):
        self.uav0.wait_for_topics(60)
        self.uav1.wait_for_topics(60)
        self.uav2.wait_for_topics(60)

        self.uav0.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)
        self.uav1.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)
        self.uav2.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)

        self.uav0.log_topic_vars()
        self.uav1.log_topic_vars()
        self.uav2.log_topic_vars()

        self.uav0.set_mode("OFFBOARD", 5)
        self.uav1.set_mode("OFFBOARD", 5)
        self.uav2.set_mode("OFFBOARD", 5)

        self.uav0.set_arm(True, 5)
        self.uav1.set_arm(True, 5)
        self.uav2.set_arm(True, 5)

        rospy.loginfo("run mission")
        positions0 = ((0, 0, -0.5), (0, 0, 20), (-75, 0, 26), (-75, 0, 0))
        positions1 = ((0, 0, -0.3), (0, 0, 20), (-75, 0, 26), (-75, 0, 0))
        positions2 = ((0, 0, -0.1), (0, 0, 20), (-75, 0, 26), (-75, 0, 0))

        for i in xrange(len(positions0)):
            self.reach_position(positions0[i], positions1[i],positions2[i], 30)


        self.uav0.set_mode("AUTO.LAND", 5)
        self.uav1.set_mode("AUTO.LAND", 5)
        self.uav2.set_mode("AUTO.LAND", 5)
        self.uav0.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                    45, 0)
        self.uav0.set_arm(False, 5)



if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)

    # rostest.rosrun(PKG, 'mavros_offboard_posctl_test', MavrosOffboardPosctlTest)

    MAV = MavrosOffboardPosctlTest()
    MAV.test_posctl()

