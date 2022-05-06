#!/usr/bin/env python3
#
# Software License Agreement (Apache 2.0 License)
# Copyright (c) 2022, The Ohio State University
#
# Author: C. Cooper
#
# Description:
# Base swing model. Exactly follows the Super Points which shape the base swing.
# Builds trajectory using waypoints and executes motion to robot.


###########
# Imports #
###########

import numpy as np

import matplotlib.pyplot as plt
from matplotlib import animation, rc
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D

from smart_golf_utilities import ROBOT_KINEMATICS, drawRobot2, Interp
from trajectory_action_client import SimpleTrajectoryActionClient


######################
## Swing Parameters ##
######################

index = np.arange(0,7,1)
# print(index)
# S = [-np.pi/2,-np.pi/3,-np.pi/6,0,np.pi/6,np.pi/3,np.pi/2]
# L = [.0799,.4289,.7338,.8011,.7616,.7616,.6299]
# U = [.5850,.5085,-.1047,-.1047,.1899,.8684,1.133]
# R = [2.492,2.499,2.019,np.pi/2,1.477,.6686,.6686]
# B = [-1.229,-1.225,-1.246,0,.6196,1.362,1.526]
# T = [np.pi,np.pi,np.pi,np.pi,np.pi,np.pi,np.pi]

# NEW SUPER POINTS
S = [-np.pi/2,-np.pi/3,-np.pi/6,0,np.pi/6,np.pi/3,np.pi/2-.2]
L = [.0799,.4289,.7338,.8011,.7617,.7616,.6299]
U = [.5850,.250,-.1047,-.1047,.1899,.8684,1.133]
R = [2.492,2.250,2.019,np.pi/2,1.277,.6686,.6686]
B = [-1.229,-1.225,-.950,0,.6196,1.362,1.526]
T = [np.pi,np.pi,np.pi,np.pi,np.pi,np.pi,np.pi]


#Plot them all
# creating an empty canvas
fig = plt.figure(figsize = (15,10))
ax = fig.add_subplot(1,1,1)
ax.plot(index,S,'-o',index,L,'-o',index,U,'-o',index,R,'-o',index,B,'-o',index,T,'-o')
plt.legend(['S','L','U','R','B','T'],fontsize = 16)
ax.set_title('Human Swing Joint Space',fontsize = 16)
ax.set_xlabel('Time',fontsize = 14)
ax.set_ylabel('radians',fontsize = 14)
plt.show()



##########
## Main ##
##########

intervals = 16

long_S = Interp(S,intervals)
long_L = Interp(L,intervals)
long_U = Interp(U,intervals)
long_R = Interp(R,intervals)
long_B = Interp(B,intervals)
long_T = Interp(T,intervals)
                     
pts = len(long_R)
print(len(long_R))
print(long_R)

t_start = 0
t_end =3
delta_t = t_end/pts
t_pts = np.arange(t_start,t_end,delta_t)

fig2 = plt.figure(figsize = (14,7))
ax2 = fig2.add_subplot(1,2,1)
ax3 = fig2.add_subplot(1,2,2)
ax2.plot(t_pts,long_S,'-')
ax2.plot(t_pts,long_L,'-')
ax2.plot(t_pts,long_U,'-')
ax2.plot(t_pts,long_R,'-')
ax2.plot(t_pts,long_B,'-')
ax2.plot(t_pts,long_T,'-')
ax3.plot(index,S,'-o')
ax3.plot(index,L,'-o')
ax3.plot(index,U,'-o')
ax3.plot(index,R,'-o')
ax3.plot(index,B,'-o')
ax3.plot(index,T,'-o')
plt.legend(['S','L','U','R','B','T'],fontsize = 16)
plt.show()

# plt.show()
plt.savefig("fig2_jointPosn_fromInterp.png")


# Make a real robot 

real_links = np.array([88*np.sqrt(2),400,405,876,50.8])
links_in = real_links*(1/25.4)
#print(links)
axis = [[0,0,1],[0,1,0],[0,1,0],[0,1,0],[1,0,0]]

# NEW SUPER POINTS
S = [-np.pi/2,-np.pi/3,-np.pi/6,0,np.pi/6,np.pi/3,np.pi/2-.2]
L = [.0799,.4289,.7338,.8011,.7617,.7616,.6299]
U = [.5850,.250,-.1047,-.1047,.1899,.8684,1.133]
R = [2.492,2.250,2.019,np.pi/2,1.277,.6686,.6686]
B = [-1.229,-1.225,-.950,0,.6196,1.362,1.526]
T = [np.pi,np.pi,np.pi,np.pi,np.pi,np.pi,np.pi]


#anglesDesired = [np.pi/4,0,0,np.pi/2,np.pi/2,0]
anglesDesired = [S[3],L[3],U[3],R[3],B[3]-np.pi/4,T[3]]
#anglesDesired = [0,.5,.5,10,1,10]

#Changing from normal physics conventions of angle definition to the way the robot defines them:
anglesConvention = [-1*anglesDesired[0],(np.pi/4-anglesDesired[1]),\
                    (-np.pi/2+anglesDesired[2]),anglesDesired[3],anglesDesired[4],np.pi-anglesDesired[5]]

Moto = ROBOT_KINEMATICS(links = links_in, axis = axis)
End = Moto.findEnd(anglesConvention)
#print([End[0],End[1],End[2]])

#Show on a 3D plot

v1,v2,v3,v4,v5 = End[3],End[4],End[5],End[6],End[7]

x,y,z = drawRobot2(v1,v2,v3,v4,v5)
             
#print(x)
#print(y)
#print(z) 


# creating an empty canvas
fig = plt.figure(figsize = (7,7))

# defining the axes with the projection
# as 3D so as to plot 3D graphs
ax = plt.axes(projection="3d")
ax.set_xlim(0,60)
ax.set_ylim(-30,30)
ax.set_zlim(-30,20)

# plotting a 3D line graph with X-coordinate,
# Y-coordinate and Z-coordinate respectively
ax.plot3D(x,y,z, 'red')


 
# Showing the above plot
plt.title('Model Orientation of Robot',fontsize=13)
plt.show()

x_end = np.zeros(pts,)
y_end = np.zeros(pts,)
z_end = np.zeros(pts,)
dist = np.zeros(pts-1,)

for i in range(pts):
    
    anglesDesired = [long_S[i],long_L[i],long_U[i],long_R[i],long_B[i],long_T[i]]
    #Changing from normal physics conventions of angle definition to the way the robot defines them:
    anglesConvention = [-1*anglesDesired[0],(np.pi/4-anglesDesired[1]),\
                        (-np.pi/2+anglesDesired[2]),anglesDesired[3],anglesDesired[4],np.pi-anglesDesired[5]]
    
    End = Moto.findEnd(anglesConvention)
    x_end[i],y_end[i],z_end[i] = End[0],End[1],End[2]

    
    
#print(x_end)

# creating an empty canvas
fig2 = plt.figure(figsize = (7,7))

# defining the axes with the projection
# as 3D so as to plot 3D graphs
ax2 = plt.axes(projection="3d")
ax2.set_xlim(0,60)
ax2.set_ylim(-30,30)
ax2.set_zlim(-30,20)
# plotting a 3D line graph with X-coordinate,
# Y-coordinate and Z-coordinate respectively
ax2.plot3D(x_end,y_end,z_end, 'red')
# Showing the above plot
plt.title('Swing Path',fontsize=13)
plt.show()




def makeTime(x_end,y_end,z_end,max_vel):
    #Initalize Everything
    pts = len(x_end)
    time = 0
    t_array = np.array([0])
    vel = np.zeros(pts,)
    dist = np.zeros(pts-1,)
    
    #Make the velocity of the actual club a gauss curve
    for i in range(len(t_pts-1)):
        sigma = 20
        vel[i] = max_vel*np.exp(-(i-(pts/2))**2/(2*sigma**2))
        
    for i in range(pts-1):
        dist[i] = np.sqrt((x_end[i+1]-x_end[i])**2+(y_end[i+1]-y_end[i])**2+(z_end[i+1]-z_end[i])**2)
        time = time + dist[i]/vel[i]
        t_array = np.append(t_array,time)
        
    return t_array,vel,dist
   
    
max_vel = 36*12 #in/sec
t_array,vel,dist = makeTime(x_end,y_end,z_end,max_vel)

print('Printing: t_array, size(t_array), vel, dist')
print(t_array)
print(np.size(t_array))
print(vel)
print(dist)

#Find the joint velocities
vel_s = [0]
vel_l = [0]
vel_u = [0]
vel_r = [0]
vel_b = [0]
vel_t = [0]

for i in range(int(len(long_S)-1)):
    vel_s = np.append(vel_s,(long_S[i+1]-long_S[i])/(t_array[i+1]-t_array[i]))
    vel_l = np.append(vel_l,(long_L[i+1]-long_L[i])/(t_array[i+1]-t_array[i]))
    vel_u = np.append(vel_u,(long_U[i+1]-long_U[i])/(t_array[i+1]-t_array[i]))
    vel_r = np.append(vel_r,(long_R[i+1]-long_R[i])/(t_array[i+1]-t_array[i]))
    vel_b = np.append(vel_b,(long_B[i+1]-long_B[i])/(t_array[i+1]-t_array[i]))
    vel_t = np.append(vel_t,(long_T[i+1]-long_T[i])/(t_array[i+1]-t_array[i]))

print('Printing vel_s then vel_b then vel_u')
print(vel_s)
print(vel_b)
print(vel_u)

fig4 = plt.figure(figsize = (14,7))
ax4 = fig4.add_subplot(1,2,1)
ax5 = fig4.add_subplot(1,2,2)
ax4.plot(t_array,long_S,'-')
ax4.plot(t_array,long_L,'-')
ax4.plot(t_array,long_U,'-')
ax4.plot(t_array,long_R,'-')
ax4.plot(t_array,long_B,'-')
ax4.plot(t_array,long_T,'-')
ax5.plot(index,S,'-o')
ax5.plot(index,L,'-o')
ax5.plot(index,U,'-o')
ax5.plot(index,R,'-o')
ax5.plot(index,B,'-o')
ax5.plot(index,T,'-o')
plt.legend(['S','L','U','R','B','T'],fontsize = 16)
plt.show()



# Swinging:
joint_names = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
Joint_START = [long_S[0],long_L[0],long_U[0],long_R[0],long_B[0],long_T[0]]
t_start = 3
# go to start
traj_plan_start = SimpleTrajectoryActionClient(joint_names)
traj_plan_start.add_joint_waypoint(Joint_START,t_start,[0,0,0,0,0,0])
traj_plan_start.send_trajectory()

half_swing = int(len(long_S)/2)
t_back = np.linspace(2,7,half_swing)


traj_plan_swing = SimpleTrajectoryActionClient(joint_names)
for i in range(len(t_array)):
    traj_plan_swing.add_joint_waypoint([long_S[i],long_L[i],long_U[i],long_R[i],long_B[i],long_T[i]],t_array[i]+t_start,[vel_s[i],vel_l[i],vel_u[i],vel_r[i],vel_b[i],vel_t[i]])
for j in range(int(len(t_array)/2)-1):
    j+=1
    traj_plan_swing.add_joint_waypoint([long_S[-j],long_L[-j],long_U[-j],long_R[-j],long_B[-j],long_T[-j]],t_array[-1]+t_start+t_back[j],[0,0,0,0,0,0])

traj_plan_swing.add_joint_waypoint([long_S[40],long_L[40],long_U[40],long_R[40],long_B[40],long_T[40]],t_array[-1]+t_start+t_back[j]+2,[0,0,0,0,0,0])

traj_plan_swing.send_trajectory()