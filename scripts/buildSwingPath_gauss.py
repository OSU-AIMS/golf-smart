#!/usr/bin/env python3
#
# Software License Agreement (Apache 2.0 License)
# Copyright (c) 2022, The Ohio State University
#
# Author: C. Cooper
#
# Description:
# This script contains the original GAUSSIAN MODEL method created by the 2021-22 Robotic Golf Capstone team
# Only the S and B joints are modified to create a path of specific position and velocity through time.


###########
# Imports #
###########

import numpy as np

import matplotlib.pyplot as plt
from matplotlib import animation, rc
from mpl_toolkits import mplot3d
import random as random
from mpl_toolkits.mplot3d import Axes3D

from trajectory_action_client import SimpleTrajectoryActionClient


#####################
# Support Functions #
#####################

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



#####################
# Guass Swing Model #
#####################

def gaussian_swing_define(Joint_START, Joint_END, sigmas, swing_time, delta_t):

    t_start = 5

    # go to start


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
    

    ################################
    ## Plotting Generate Path Plan

    # Setup Figure
    fig4= plt.figure(figsize = (7,4))
    fig4.suptitle('Joint Path Plan using a Gauss Model', fontsize='large', fontweight='bold')

    # Define Axis
    ax_S2 = fig4.add_subplot(1,2,1)
    ax_B2 = fig4.add_subplot(1,2,2)

    # Plotting the velocity and positions on each other
    # 'S' Joint
    ax_S2.plot(t_pts, vel_s, 'red')
    ax_S2.plot(t_pts, S,     'blue')
    ax_S2.set_title('S Joint')
    ax_S2.legend(['Velocity', 'Position'])

    # 'B' Joint
    ax_B2.plot(t_pts, vel_b, 'red')
    ax_B2.plot(t_pts, B,     'blue')
    ax_B2.set_title('B Joint')
    ax_B2.legend(['Velocity', 'Position'])

    # Export Figure
    plt.savefig('pathPlan_GaussModel.png')

    return traj_plan_swing



########
# Main #
########

if __name__ == '__main__':


    ## Build Swing Path Plan (and Trajectory)

    # Robot Parameters
    # joint_names = rospy.get_param('controller_joint_names')
    joint_names = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']

    # Path Start/End Joint Positions 
    Joint_START = [-np.pi/4, np.pi/24, -np.pi/4, np.pi/2, -np.pi/3, np.pi]
    Joint_END   = [ np.pi/4, np.pi/24, -np.pi/4, np.pi/2,  np.pi/3, np.pi]

    # Path Shape Parameters
    sigmas = [.8,0,0,0,.15,0]
    swing_time = 8
    delta_t = swing_time/100

    # Generate Swing Path
    swing_trajectory = gaussian_swing_define(Joint_START, Joint_END, sigmas, swing_time, delta_t)


    ## Execute Path Plan
    
    # Move to All-Zeros
    traj_start = SimpleTrajectoryActionClient(joint_names)
    traj_start.add_joint_waypoint(Joint_START, 5, [0,0,0,0,0,0])
    traj_start.send_trajectory()

    # Send Full Trajectory
    # swing_trajectory.send_trajectory()

#EOF