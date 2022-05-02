import numpy as np

import matplotlib.pyplot as plt
from matplotlib import animation, rc
from mpl_toolkits import mplot3d
import random as random
from mpl_toolkits.mplot3d import Axes3D

from trajectory_action_client import SimpleTrajectoryActionClient

import sys

"""
The point of the tophat swing is to lengthing the amount of time a joint is at its max velocity compared to the gaussian swing
The tophat is built with two sigmoid functions flipped on top of each other.

You are able to define the amplituide and the time for the swing to occur. 
The width is calculated so that the integral matches how much angle needs to be swept out
"""
def sigmoid(amp,width,center,t_pts,mag_vel):
    vel = np.zeros(len(t_pts))
    n = 10*mag_vel
    half = len(t_pts)/2
    right = center-width
    for i in range(int(half)):
        vel[i] = amp/(1+np.exp(-n*(t_pts[i]-right)))
    for j in range(int(half),len(t_pts)):
        vel[j] = amp/(1+np.exp(-n*(-t_pts[j]+(2*width+right))))
    return vel


# This is the integral of the tophat. It is actually exact since there is some nic symmetry 
def sig_int(amp,width):
    integral = amp*width*2
    return integral

# Sets the width 
def find_width(amp,delta_theta):
    width = delta_theta/(amp*2)
    return width

# Numerical integral to find theta from velocity
def num_integrate(vel,t_pts):
    area = 0
    delta_x = t_pts[1]-t_pts[0]
    theta = np.zeros(len(vel))
    
    for i in range(len(vel)):
        area = area + vel[i]*delta_x
        theta[i] = area
        
    return theta 


def tophat_swing_define(Joint_START,Joint_END,amp_s,amp_b,swing_time,delta_t):


    mag_vel = np.sqrt(amp_s**2+amp_b**2) # This is to keep the 'n' variable in check
    t_start = 2 # time to go back to zero
    # go to start
    traj_plan_start = SimpleTrajectoryActionClient(joint_names)
    traj_plan_start.add_joint_waypoint(Joint_START,t_start,[0,0,0,0,0,0])
    traj_plan_start.send_trajectory()

    t_0 = 0
    t_end = swing_time
    t_pts = np.arange(t_0,t_end,delta_t)

   # Hips move Tophat S
    S_start = Joint_START[0]
    S_end = Joint_END[0]
    S_delta_theta = (S_end-S_start)
    center_s = swing_time/2.3
    width_s = find_width(amp_s,S_delta_theta)
    vel_s = sigmoid(amp_s,width_s,center_s,t_pts,mag_vel)
    S = S_start + num_integrate(vel_s,t_pts)
    
    # Wrist Moves Tophat B
    B_start = Joint_START[4]
    B_end = Joint_END[4]
    B_delta_theta = (B_end-B_start)
    center_b = swing_time/2
    width_b = find_width(amp_b,B_delta_theta)
    vel_b = sigmoid(amp_b,width_b,center_b,t_pts,mag_vel)
    B = B_start + num_integrate(vel_b,t_pts)


    #All other joints do not move 
    L = np.ones(len(t_pts))*Joint_START[1]
    U = np.ones(len(t_pts))*Joint_START[2]
    R = np.ones(len(t_pts))*Joint_START[3]
    T = np.ones(len(t_pts))*Joint_START[5]
    vel_l = np.ones(len(t_pts))*0
    vel_u = np.ones(len(t_pts))*0
    vel_r = np.ones(len(t_pts))*0
    vel_t = np.ones(len(t_pts))*0

    #Send the trajectory
    traj_plan_swing = SimpleTrajectoryActionClient(joint_names)
    for i in range(len(t_pts)):
        traj_plan_swing.add_joint_waypoint([S[i],L[i],U[i],R[i],B[i],T[i]],t_pts[i]+t_start+1,[vel_s[i],vel_l[i],vel_u[i],vel_r[i],vel_b[i],vel_t[i]])
    traj_plan_swing.send_trajectory()
    print(B)
    print(S)


    fig4= plt.figure(figsize = (14,7))

    # defining the axis
    ax_S2 = fig4.add_subplot(1,2,1)
    ax_B2 = fig4.add_subplot(1,2,2)

    #Plotting the velocity and positions on each other
    ax_S2.plot(t_pts,vel_s,'red')
    ax_S2.plot(t_pts,S,'blue')
    ax_S2.set_title('S Joint')
    ax_B2.plot(t_pts,vel_b,'red')
    ax_B2.plot(t_pts,B,'blue')
    ax_B2.set_title('B Joint')

    # Showing the above plot
    plt.legend(['Velocity','Position'])
    plt.show()





## Main

joint_names = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']

## tophat curve
Joint_START = [-np.pi/2,np.pi/24,-np.pi/4,np.pi/2,-np.pi/3,np.pi]
Joint_END = [np.pi/2,np.pi/24,-np.pi/4,np.pi/2,np.pi/3,np.pi]
amp_s = 4
amp_b = 7
swing_time = 2
delta_t = swing_time/1000
tophat_swing_define(Joint_START,Joint_END,amp_s,amp_b,swing_time,delta_t)
        
