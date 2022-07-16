#! /usr/bin/env python

import rospy
import rospkg
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion

import threading
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState 

from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import LaserScan
#from kobuki_msgs.msg import BumperEvent
import time

import tensorflow
from keras.models import Sequential, Model
from keras.layers import Dense, Dropout, Input, merge
from keras.layers.merge import Add, Concatenate
from keras.optimizers import Adam
import keras.backend as K
import gym
import numpy as np
import math
import random

from std_srvs.srv import Empty

class InfoGetter(object):
    def __init__(self):
        self._event = threading.Event()
        self._msg = None

    def __call__(self, msg):
        self._msg = msg
        self._event.set()

    def get_msg(self, timeout=None):
        self._event.wait(timeout)
        return self._msg


class GameState:
    def __init__(self):
        self.talker_node = rospy.init_node('talker', anonymous=True)
        self.pose_ig = InfoGetter()
        self.laser_ig = InfoGetter()
        self.collision_ig = InfoGetter()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.position = Point()
        self.move_cmd = Twist()
        self.pose_info = rospy.Subscriber("/gazebo/model_states", ModelStates, self.pose_ig)
        self.laser_info = rospy.Subscriber("/laserscan_filtered", LaserScan, self.laser_ig)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")
        (self.position, self.rotation) = self.get_odom()

  
        self.rate = rospy.Rate(100) 
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0.6 
        self.move_cmd.angular.z = 0.2 
        self.crash_indicator = 0
        self.state_num = 28 #685  
        self.action_num = 2
        self.observation_space = np.empty(self.state_num)
        self.action_space = np.empty(self.action_num)
        self.laser_reward = 0

        self.target_x = 10
        self.target_y = 10

        self.model_index = 10 

        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)


    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])


    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


    def print_odom(self):
        while True:
            (self.position, self.rotation) = self.get_odom()
            print("position is %s, %s, %s, ", self.position.x, self.position.y, self.position.z)
            print("rotation is %s, ", self.rotation)


    def reset(self):
        index_list = [-1, 1]
        index_x = random.choice(index_list)
        index_y = random.choice(index_list)

        self.target_x = (np.random.random()-0.5)*5 + 12*index_x
        self.target_y = (np.random.random()-0.5)*5 + 12*index_y

        random_turtlebot_y = (np.random.random())*5 

        self.crash_indicator = 0

        state_msg = ModelState()    
        state_msg.model_name = 'turtlebot3_waffle_pi'
        state_msg.pose.position.x = 4*np.random.random()-2
        state_msg.pose.position.y = 4*np.random.random()-2 
        state_msg.pose.position.z = 0.0
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = -0.2
        state_msg.pose.orientation.w = 0

        state_target_msg = ModelState()    
        state_target_msg.model_name = 'unit_sphere_0_0'
        state_target_msg.pose.position.x = self.target_x
        state_target_msg.pose.position.y = self.target_y
        state_target_msg.pose.position.z = 0.0
        state_target_msg.pose.orientation.x = 0
        state_target_msg.pose.orientation.y = 0
        state_target_msg.pose.orientation.z = -0.2
        state_target_msg.pose.orientation.w = 0


        rospy.wait_for_service('gazebo/reset_simulation')
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
            resp_target = set_state( state_target_msg )

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        initial_state = np.ones(self.state_num)
        initial_state[self.state_num-1] = 0
        initial_state[self.state_num-2] = 0
        initial_state[self.state_num-3] = 0
        initial_state[self.state_num-4] = 0

        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.pub.publish(self.move_cmd)
        time.sleep(1)
        self.pub.publish(self.move_cmd)
        self.rate.sleep()


        return initial_state


    def turtlebot_is_crashed(self, laser_values, range_limit):
        self.laser_crashed_value = 0
        self.laser_crashed_reward = 0

        for i in range(len(laser_values)):
            if (laser_values[i] < 2*range_limit):
                self.laser_crashed_reward = -80
            if (laser_values[i] < range_limit):
                self.laser_crashed_value = 1
                self.laser_crashed_reward = -200
                self.reset()
                time.sleep(1)
                break
        return self.laser_crashed_reward


    def game_step(self, time_step=0.1, linear_x=0.8, angular_z=0.3, noise_level = 0):


        start_time = time.time()
        record_time = start_time
        record_time_step = 0
        self.move_cmd.linear.x = linear_x*0.26
        self.move_cmd.angular.z = angular_z
        self.rate.sleep()


        (self.position, self.rotation) = self.get_odom()
        turtlebot_x_previous = self.position.x
        turtlebot_y_previous = self.position.y


        while (record_time_step < time_step) and (self.crash_indicator==0):
            self.pub.publish(self.move_cmd)
            self.rate.sleep()
            record_time = time.time()
            record_time_step = record_time - start_time

        (self.position, self.rotation) = self.get_odom()
        turtlebot_x = self.position.x
        turtlebot_y = self.position.y
        angle_turtlebot = self.rotation
        angle_turtlebot_target = atan2(self.target_y - turtlebot_y, self.target_x- turtlebot_x)

        if angle_turtlebot < 0:
            angle_turtlebot = angle_turtlebot + 2*math.pi

        if angle_turtlebot_target < 0:
            angle_turtlebot_target = angle_turtlebot_target + 2*math.pi


        angle_diff = angle_turtlebot_target - angle_turtlebot
        if angle_diff < -math.pi:
            angle_diff = angle_diff + 2*math.pi
        if angle_diff > math.pi:
            angle_diff = angle_diff - 2*math.pi

        # prepare the normalized laser value and check if it is crash
        laser_msg = self.laser_ig.get_msg()
        laser_values = laser_msg.ranges + 2 * noise_level * np.random.rand(24) - noise_level
        received_laser_values = np.clip(laser_values, 0.12, 3.5)
        normalized_laser = [(x) / 3.5 for x in received_laser_values]
        normalized_laser = normalized_laser
        current_distance_turtlebot_target = math.sqrt((self.target_x - turtlebot_x)**2 + (self.target_y - turtlebot_y)**2)
        state = np.append(normalized_laser, current_distance_turtlebot_target)
        state = np.append(state, angle_diff)
        state = np.append(state, linear_x * 0.26)
        state = np.append(state, angular_z)
        state = state.reshape(1, self.state_num)

        (self.position, self.rotation) = self.get_odom()
        turtlebot_x = self.position.x
        turtlebot_y = self.position.y
        distance_turtlebot_target_previous = math.sqrt((self.target_x - turtlebot_x_previous)**2 + (self.target_y - turtlebot_y_previous)**2)
        distance_turtlebot_target = math.sqrt((self.target_x - turtlebot_x)**2 + (self.target_y - turtlebot_y)**2)
        distance_reward = distance_turtlebot_target_previous - distance_turtlebot_target

        self.laser_crashed_reward = self.turtlebot_is_crashed(laser_values, range_limit=0.25)
        self.laser_reward = sum(normalized_laser)-24
        self.collision_reward = self.laser_crashed_reward + self.laser_reward


        self.angular_punish_reward = 0
        self.linear_punish_reward = 0

        if angular_z > 0.8:
            self.angular_punish_reward = -1
        if angular_z < -0.8:
            self.angular_punish_reward = -1

        if linear_x < 0.2:
            self.linear_punish_reward = -2


        self.arrive_reward = 0
        if distance_turtlebot_target<1:
            self.arrive_reward = 100
            self.laser_crashed_value = 2
            self.reset()
            time.sleep(1)

        if min(laser_values) > 3:
            self.level = 0
        elif min(laser_values) > 2:
            self.level = 1
        elif min(laser_values) > 1:
            self.level = 2
        elif min(laser_values) > 0.25:
            self.level = 3
        else:
            self.level = 4  # crashed
        # if distance_turtlebot_target >3.5:
        #     if min(laser_values) > 3:
        #         self.level = 0
        #     elif min(laser_values) > 2:
        #         self.level = 1
        #     elif min(laser_values) > 1:
        #         self.level = 2
        #     elif min(laser_values) > 0.25:
        #         self.level = 3
        #     else:
        #         self.level = 4 #crashed
        # else:
        #     if min(laser_values)>distance_turtlebot_target-0.25 and min(laser_values)<distance_turtlebot_target+0.25:
        #         self.level = 0
        #     elif min(laser_values) > 3:
        #         self.level = 0
        #     elif min(laser_values) > 2:
        #         self.level = 1
        #     elif min(laser_values) > 1:
        #         self.level = 2
        #     elif min(laser_values) > 0.25:
        #         self.level = 3
        #     else:
        #         self.level = 4


        reward  = distance_reward*(5/time_step)*1.2*7 + self.arrive_reward + self.collision_reward + self.angular_punish_reward + self.linear_punish_reward

        return reward, state, self.laser_crashed_value, self.level




