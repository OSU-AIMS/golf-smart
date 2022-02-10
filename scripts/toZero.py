#!/usr/bin/env python
# coding: utf-8

# In[1]:



# In[66]:


import numpy as np
import scipy.integrate as integrate
from scipy.integrate import odeint, solve_ivp

import matplotlib.pyplot as plt
from matplotlib import animation, rc
from mpl_toolkits import mplot3d
import random as random
from mpl_toolkits.mplot3d import Axes3D
from IPython.display import HTML

from trajectory_action_client import SimpleTrajectoryActionClient

import csv
import os

import rospy
import actionlib
import time
import math

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

joint_names = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']


# go to start
traj_plan_start = SimpleTrajectoryActionClient(joint_names)
traj_plan_start.add_joint_waypoint([0,0,0,0,0,0],3,[0,0,0,0,0,0])
traj_plan_start.send_trajectory()