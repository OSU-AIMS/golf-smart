#!/usr/bin/env python3
#
# Software License Agreement (Apache 2.0 License)
# Copyright (c) 2021, The Ohio State University
# The Artificially Intelligent Manufacturing Systems Lab (AIMS)
#
# Author: acbuynak and mschoenleb


from trajectory_action_client import SimpleTrajectoryActionClient
import numpy as np
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

def main():
    # Parameters
    
    # Setup Control Client
    traj_plan = SimpleTrajectoryActionClient(joint_names)
    traj_plan.add_joint_waypoint([-.8,.2,-.75,0,0,0],2,[.05,.1,.05,.0,.0,.0])
    traj_plan.send_trajectory()
    rospy.loginfo("ran the stuff")

def reset_joints():
    # Parameters
    
    # Setup Control Client
    traj_plan = SimpleTrajectoryActionClient(joint_names)
    traj_plan.add_joint_waypoint([0,0,-.75,0,0,0],4,[-.05,.0,.05,.0,.0,.0])
    traj_plan.send_trajectory()
    rospy.loginfo("ran the stuff")

def reset_joints_safe():
    # Parameters
    
    # Setup Control Client
    traj_plan = SimpleTrajectoryActionClient(joint_names)
    traj_plan.add_joint_waypoint([-1.57,0,0,0,0,0],4,[-.05,.0,.0,.0,.0,.0])
    traj_plan.send_trajectory()
    rospy.loginfo("ran the stuff")


if __name__ == '__main__':
    Joint_START = [-np.pi/2,np.pi/24,-np.pi/4,np.pi/2,-np.pi/2,-np.pi/2]
    main()
    reset_joints()
