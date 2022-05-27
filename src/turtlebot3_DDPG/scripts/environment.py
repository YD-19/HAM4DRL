#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn

class Env():
    def __init__(self):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        # self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.state_num = 28
        self.action_num = 2
        self.observation_space = np.empty(self.state_num)
        self.action_space = np.empty(self.action_num)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn()

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)

        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 2)

    def getState(self, scan):
        scan_range = []
        heading = self.heading
        min_range = 0.15
        done = False

        self.laser_crashed_reward = 0
        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(1.0)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        # print("scan is %s", min(scan_range))

        if 2*min_range > min(scan_range) > 0:
            self.laser_crashed_reward = -80

        if min_range > min(scan_range) > 0:
            done = True
            rospy.loginfo("Collision!!")
            self.laser_crashed_reward = -200
            self.reset()
            # self.get_goalbox = False
        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        self.arrive_reward = 0
        if current_distance < 0.5:
            self.get_goalbox = True
            # if self.get_goalbox:
            rospy.loginfo("Goal!!")
            self.arrive_reward = 100
            # self.pub_cmd_vel.publish(Twist())
            self.reset()
            self.get_goalbox = False




        return scan_range + [heading, current_distance], done
    # def turtlebot_is_crashed(self, laser_values, range_limit):
    #     self.laser_crashed_value = 0
    #     self.laser_crashed_reward = 0
    #
    #     for i in range(len(laser_values)):
    #         if (laser_values[i] < 2*range_limit):
    #             self.laser_crashed_reward = -80
    #         if (laser_values[i] < range_limit):
    #             self.laser_crashed_value = 1
    #             self.laser_crashed_reward = -200
    #             self.reset()
    #             # time.sleep(1)
    #             break
    #     return self.laser_crashed_reward

    def setReward(self, state, action):

        time_step = 0.1

        # self.laser_crashed_reward = self.turtlebot_is_crashed(state[0:24], range_limit=0.25)
        self.laser_reward = sum(state[0:24]) - 24
        self.collision_reward = self.laser_crashed_reward + self.laser_reward

        self.angular_punish_reward = 0
        self.linear_punish_reward = 0

        if action[0] > 0.8:
            self.angular_punish_reward = -1
        if action[0] < -0.8:
            self.angular_punish_reward = -1

        if action[1] < 0.2:
            self.linear_punish_reward = -2

        # self.arrive_reward = 0
        # if distance_turtlebot_target < 1:
        #     self.arrive_reward = 100
        #     self.reset()
        #     time.sleep(1)



        reward = self.distance_reward * (
                    5 / time_step) * 1.2 * 7 + self.arrive_reward + self.collision_reward + self.angular_punish_reward + self.linear_punish_reward

        # yaw_reward = []
        # current_distance = state[-1]
        # heading = state[-2]
        #
        # for i in range(5):
        #     angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
        #     tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
        #     yaw_reward.append(tr)
        #
        # distance_rate = 2 ** (current_distance / self.goal_distance)
        # reward = ((round(yaw_reward[action[1]] * 5, 2)) * distance_rate)
        #
        # if done:
        #     rospy.loginfo("Collision!!")
        #     reward = -200
        #     self.pub_cmd_vel.publish(Twist())
        #


        return reward

    def step(self, action):
        # max_angular_vel = 1.5
        # ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5
        self.previous_distance = self.currentdistance
        vel_cmd = Twist()
        vel_cmd.linear.x = action[1] * 0.26
        vel_cmd.angular.z = action[0]
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, done = self.getState(data)
        self.currentdistance = state[-1]

        self.distance_reward = self.previous_distance - self.currentdistance
        reward = self.setReward(state, action)

        return np.hstack([np.asarray(state),action]), reward, done

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False
        self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
        self.goal_distance = self.getGoalDistace()
        # self.goal_distance = self.getGoalDistace()
        state, done = self.getState(data)
        self.currentdistance = state[-1]
        state.append(0)
        state.append(0)
        return np.asarray(state)