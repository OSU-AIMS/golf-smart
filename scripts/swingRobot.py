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
import sys
import rospy
import actionlib
import time
import math

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

## Functions
## Original Gaussian Curve Model

def Gauss(t_pts, sigma, mean, amp ):
    f = np.zeros(len(t_pts))
    time_end = t_pts[-1]+(t_pts[1]-t_pts[0])
    max_sig = time_end/6
    sigma_adj = sigma*max_sig
    for i in range(len(t_pts)):
        f[i] = amp*np.exp(-(t_pts[i]-mean)**2/(2*sigma_adj**2))
    
    return f



def num_integrate(vel,t_pts):
    area = 0
    delta_x = t_pts[1]-t_pts[0]
    theta = np.zeros(len(vel))
    
    for i in range(len(vel)):
        area = area + vel[i]*delta_x
        theta[i] = area
        
    return theta 


def find_amp(sigma,delta_theta,t_pts):
    time_end = t_pts[-1]+(t_pts[1]-t_pts[0])
    max_sig = time_end/6
    sigma_adj = sigma*max_sig
    amp = delta_theta/(sigma_adj*np.sqrt(2*np.pi))
    return amp


def gaussian_swing_define(Joint_START,Joint_END,sigmas,swing_time,delta_t):

    t_start = 5
    # go to start
    traj_plan_start = SimpleTrajectoryActionClient(joint_names)
    traj_plan_start.add_joint_waypoint(Joint_START,t_start,[0,0,0,0,0,0])
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
    mean_s = swing_time/2.15
    amp_s = find_amp(sigma_s, S_delta_t,t_pts)
    vel_s = Gauss(t_pts,sigma_s,mean_s,amp_s)
    S = S_start + num_integrate(vel_s,t_pts)

    sigma_b = sigmas[4]
    mean_b = swing_time/2
    amp_b = find_amp(sigma_b, B_delta_t,t_pts)
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
        traj_plan_swing.add_joint_waypoint([S[i],L[i],U[i],R[i],B[i],T[i]],t_pts[i]+t_start+1,[vel_s[i],vel_l[i],vel_u[i],vel_r[i],vel_b[i],vel_t[i]])
    traj_plan_swing.send_trajectory()


## Tophat Curve Model

def swing(amp,flatness,width,center,t_pts):
    vel = np.zeroes(len(t_pts))
    for i in range(len((t_pts))):
        vel[i] = amp/(1+abs((t_pts[i]-center)/width)**(flatness))
    return vel


def sigmoid(amp,width,center,t_pts):
    vel = np.zeros(100)
    n = 100/t_pts[-1]
    half = len(t_pts)/2
    right = center-width
    for i in range(int(half)):
        vel[i] = amp/(1+np.exp(-n*(t_pts[i]-right)))
    for j in range(int(half),len(t_pts)):
        vel[j] = amp/(1+np.exp(-n*(-t_pts[j]+(2*width+right))))
    return vel


def sig_int(amp,width):
    integral = amp*width*2
    return integral


def find_width(amp,delta_theta):
    width = delta_theta/(amp*2)
    return width


def tophat_swing_define(Joint_START,Joint_END,amp_s,amp_b,swing_time,delta_t):
    t_start = 0
    t_end = swing_time
    t_pts = np.arange(t_start,t_end,delta_t)

    # Hips move Gauss S
    S_start = -Joint_START[0]
    S_end = Joint_END[0]
    S_delta_theta = (S_end-S_start)
    center_s = .8
    width_s = find_width(amp_s,S_delta_theta)
    vel_s = sigmoid(amp_s,width_s,center_s,t_pts)
    S = S_start + num_integrate(vel_s,t_pts)
    
    # Wrist Moves Gauss B
    B_start = Joint_START[4]
    B_end = -Joint_END[4]
    B_delta_theta = (B_end-B_start)*-1
    center_b = 1
    width_b = find_width(amp_b,B_delta_theta)
    vel_b = -1*sigmoid(amp_b,width_b,center_b,t_pts)
    B = B_start + num_integrate(vel_b,t_pts)

    L = np.ones(len(t_pts))*Joint_START[1]
    U = np.ones(len(t_pts))*Joint_START[2]
    R = np.ones(len(t_pts))*Joint_START[3]
    T = np.ones(len(t_pts))*Joint_START[5]
    vel_l = np.ones(len(t_pts))*0
    vel_u = np.ones(len(t_pts))*0
    vel_r = np.ones(len(t_pts))*0
    vel_t = np.ones(len(t_pts))*0



## Main

joint_names = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']

#joint_names = rospy.get_param('controller_joint_names')

if __name__ == '__main__':

    if len(sys.argv)<2:
        print("Use an argument for whichever model you want to run (0 - to zero, 1 - gaussian curve, 2 - tophat curve)")
        exit(0)
    if sys.argv[1] == '0':
        # go to 0
        traj_plan_start = SimpleTrajectoryActionClient(joint_names)
        traj_plan_start.add_joint_waypoint([0,0,0,0,0,0],4,[0,0,0,0,0,0])
        traj_plan_start.send_trajectory()

    elif sys.argv[1] =='1':
        ## gaussian swing model
        Joint_START = [-np.pi/4,np.pi/24,-np.pi/4,np.pi/2,-np.pi/3,np.pi]
        Joint_END = [np.pi/4,np.pi/24,-np.pi/4,np.pi/2,np.pi/3,np.pi]
        sigmas = [.8,0,0,0,.15,0]
        swing_time = 8
        delta_t = swing_time/100
        gaussian_swing_define(Joint_START,Joint_END,sigmas,swing_time,delta_t)

    elif sys.argv[1] == '2':
        ## tophat curve
        Joint_START = [-np.pi/4,np.pi/24,-np.pi/4,np.pi/2,-np.pi/3,np.pi]
        Joint_END = [np.pi/4,np.pi/24,-np.pi/4,np.pi/2,np.pi/3,np.pi]
        amp_s = 4.3
        amp_b = 6
        swing_time = 8
        delta_t = swing_time/100
        tophat_swing_define(Joint_START,Joint_END,amp_s,amp_b,swing_time,delta_t)