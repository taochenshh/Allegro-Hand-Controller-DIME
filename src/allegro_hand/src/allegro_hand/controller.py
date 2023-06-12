# !/usr/bin/env python

# Basic imports
import os
import numpy as np
import yaml
import csv
import threading
# Other ROS imports
import rospy
from sensor_msgs.msg import JointState

# Other imports
from datetime import datetime
from copy import deepcopy as copy
from IPython import embed
from std_msgs.msg import Header
# Put proper path
from allegro_hand.utils import *

# List of all ROS Topics
JOINT_STATE_TOPIC = '/allegroHand/joint_states' 
GRAV_COMP_TOPIC = '/allegroHand/grav_comp_torques' 
COMM_JOINT_STATE_TOPIC = '/allegroHand/commanded_joint_states' 
JOINT_COMM_TOPIC = '/allegroHand/joint_cmd'

# Maximum permitted values
MAX_ANGLE = 2.1
MAX_TORQUE = 0.3

DEFAULT_VAL = None

class AllegroController(object):
    def __init__(self):
        try:
            rospy.init_node('allegro_hand_node', anonymous=True)
        except:
            pass
        self._jstate_lock = threading.RLock()
        rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self._sub_callback_joint_state)
        rospy.Subscriber(GRAV_COMP_TOPIC, JointState, self._sub_callback_grav_comp)
        rospy.Subscriber(COMM_JOINT_STATE_TOPIC, JointState, self._sub_callback_cmd__joint_state)

        self.joint_comm_publisher = rospy.Publisher(JOINT_COMM_TOPIC, JointState, queue_size=1)

        self.current_joint_pose = DEFAULT_VAL
        self.grav_comp = DEFAULT_VAL
        self.cmd_joint_state = DEFAULT_VAL
        self.jnt_names = None
        time.sleep(1)
        desired_js = JointState()
        desired_js.header = Header()
        desired_js.velocity = []
        desired_js.effort = []
        desired_js.position = []
        desired_js.name = self.jnt_names
        self.desired_js = desired_js
        
    def _sub_callback_joint_state(self, data):
        with self._jstate_lock:
            self.current_joint_pose = data
        if self.jnt_names is None:
            self.jnt_names = self.current_joint_pose.name

    def _sub_callback_grav_comp(self, data):
        self.grav_comp = data

    def _sub_callback_cmd__joint_state(self, data):
        self.cmd_joint_state = data
      
    def set_joint_positions(self, desired_action = np.zeros(16), absolute = True):
        action = self._clip(desired_action, MAX_ANGLE)
        

        if absolute is True:
            desired_angles = np.array(action)
        else:
            desired_angles = np.array(action) + self.get_joint_positions()

        self.desired_js.position = list(desired_angles)
        self.desired_js.effort = []
        self.desired_js.header.stamp = rospy.Time.now()

        self.joint_comm_publisher.publish(self.desired_js)

    def set_joint_torques(self, action=np.zeros(16)):
        action = self._clip(action, MAX_TORQUE)
        desired_torques = np.array(action)

        self.desired_js.position = []
        self.desired_js.effort = list(desired_torques)
        print('Applying the Desired Joint Torques:', self.desired_js.effort)
        self.joint_comm_publisher.publish(self.desired_js)

    def get_joint_positions(self):
        if self.current_joint_pose is None:
            print('No joint data received!')
            return
        with self._jstate_lock:
            jpos = np.array(self.current_joint_pose.position)
        return jpos

    def _clip(self, action, value):
        return np.clip(action, -value, value)

    def log_current_pose(self, log_file):
        if self.current_joint_pose is not DEFAULT_VAL:
            current_angles = self.current_joint_pose.position
            current_velocity = self.current_joint_pose.velocity
            current_torque = self.current_joint_pose.effort
        else:
            current_angles = DEFAULT_VAL
            current_velocity = DEFAULT_VAL
            current_torque = DEFAULT_VAL

        if self.grav_comp is not DEFAULT_VAL:
            grav_comp_torques = self.grav_comp.effort
        else: 
            grav_comp_torques = DEFAULT_VAL

        if self.cmd_joint_state is not DEFAULT_VAL:
            cmd_joint_position = self.cmd_joint_state.position
            cmd_joint_torque = self.cmd_joint_state.effort
        else:
            cmd_joint_position = DEFAULT_VAL
            cmd_joint_torque = DEFAULT_VAL

        time = get_datetime()

        print('Write done at:', time)

        with open(log_file, 'a') as csvfile:
            log_writer = csv.writer(csvfile, delimiter=' ')

            log_writer.writerow(
                [time]
                + [current_angles]
                + [current_velocity] 
                + [current_torque]
                + [grav_comp_torques]
                + [cmd_joint_position]
                + [cmd_joint_torque]
                )
