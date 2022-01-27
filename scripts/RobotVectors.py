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


### Robot swing control
# In[120]:
joint_names = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']

def test():
    # Parameters
    
    # Setup Control Client
    traj_plan = SimpleTrajectoryActionClient(joint_names)
    traj_plan.add_joint_waypoint([-.8,.2,-.75,0,0,0],2,[.05,.1,.05,.0,.0,.0])
    traj_plan.send_trajectory()
    

def go_to_start():
    traj_plan = SimpleTrajectoryActionClient(joint_names)
    traj_plan.add_joint_waypoint([-.8,.2,-.75,0,0,0],2,[.05,.1,.05,.0,.0,.0])
    traj_plan.send_trajectory()

def reset_joints():
    # Parameters
    
    # Setup Control Client
    traj_plan = SimpleTrajectoryActionClient(joint_names)
    traj_plan.add_joint_waypoint([0,0,-.75,0,0,0],4,[-.05,.0,.05,.0,.0,.0])
    traj_plan.send_trajectory()
    

def reset_joints_safe():
    # Parameters
    
    # Setup Control Client
    traj_plan = SimpleTrajectoryActionClient(joint_names)
    traj_plan.add_joint_waypoint([-1.57,0,0,0,0,0],4,[-.05,.0,.0,.0,.0,.0])
    traj_plan.send_trajectory()
    

def Gauss(t_pts, sigma, mean, amp ):
    f = np.zeros(len(t_pts))
    for i in range(len(t_pts)):
        f[i] = amp*np.exp(-(t_pts[i]-mean)**2/(2*sigma**2))
    
    return f


# In[121]:


def num_integrate(vel,t_pts):
    area = 0
    delta_x = t_pts[1]-t_pts[0]
    theta = np.zeros(len(vel))
    
    for i in range(len(vel)):
        area = area + vel[i]*delta_x
        theta[i] = area
        
    return theta 


# In[122]:


def find_amp(sigma,delta_theta):
    amp = delta_theta/(sigma*np.sqrt(2*np.pi))
    return amp


# In[ ]:


# THIS IS the S FUNCTION PART
def swing_define(Joint_START,Joint_END,sigmas,swing_time,delta_t):

    # go to start
    traj_plan_start = SimpleTrajectoryActionClient(joint_names)
    traj_plan_start.add_joint_waypoint(Joint_START,2,[0,0,0,0,0,0])
    traj_plan_start.send_trajectory()

    #define swing values
    S_start = Joint_START[0]
    S_end = Joint_END[0]
    S_delta_t = (S_end-S_start)
    B_start = Joint_START[4]
    B_end = Joint_END[4]
    B_delta_t = (B_end-B_start)*-1                                                                         

    t_pts = np.arange(0,swing_time,delta_t)
   
    sigma_s = sigmas[0]
    mean_s = swing_time/2
    amp_s = find_amp(sigma_s, S_delta_t)
    vel_s = Gauss(t_pts,sigma_s,mean_s,amp_s)
    S = S_start + num_integrate(vel_s,t_pts)

    sigma_b = sigmas[4]
    mean_b = swing_time/2
    amp_b = find_amp(sigma_b, B_delta_t)
    vel_b = -1*Gauss(t_pts,sigma_b,mean_b,amp_b)
    B = B_start + num_integrate(vel_b,t_pts)

    L = np.ones(len(t_pts))*Joint_START[1]
    U = np.ones(len(t_pts))*Joint_START[2]
    R = np.ones(len(t_pts))*Joint_START[3]
    T = np.ones(len(t_pts))*Joint_START[5]
    vel_l = np.ones(len(t_pts))*0
    vel_u = np.ones(len(t_pts))*0
    vel_r = np.ones(len(t_pts))*0
    vel_t = np.ones(len(t_pts))*0

    #create swing traj
    print(S)
    print(B)
    traj_plan_swing = SimpleTrajectoryActionClient(joint_names)
    print(t_pts)
    for i in range(len(t_pts)):
        traj_plan_swing.add_joint_waypoint([S[i],L[i],U[i],R[i],B[i],T[i]],t_pts[i],[vel_s[i],vel_l[i],vel_u[i],vel_r[i],vel_b[i],vel_t[i]])
    traj_plan_swing.send_trajectory()




if __name__ == '__main__':

    Joint_START = [-np.pi/2,np.pi/24,-np.pi/4,np.pi/2,-np.pi/2,-np.pi/2]
    Joint_END = [np.pi/2,np.pi/24,-np.pi/4,np.pi/2,np.pi/2,-np.pi/2]
    sigmas = [1,0,0,0,.25,0]
    swing_time = 10
    delta_t = 2
    swing_define(Joint_START,Joint_END,sigmas,swing_time,delta_t)
    #reset_joints()
