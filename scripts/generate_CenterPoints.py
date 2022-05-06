#!/usr/bin/env python3
#
# Software License Agreement (Apache 2.0 License)
# Copyright (c) 2022, The Ohio State University
#
# Author: C. Cooper
#
# Description:
# Generate ~100 different center points and export as a csv file.
# Each center point is a semi-random variation (joint-space) on the SuperPoints swing.
# Each center point results in different club angles but the same point of contact. 


#############
## Imports ##
#############

import numpy as np
import csv

import matplotlib.pyplot as plt
from matplotlib import animation, rc
from mpl_toolkits import mplot3d
import random as random
from mpl_toolkits.mplot3d import Axes3D

from smart_golf_utilities import ROBOT_KINEMATICS, drawRobot2, Interp


######################
## Swing Parameters ##
######################

index = np.arange(0,7,1)

# NEW SUPER POINTS
S = [-np.pi/2,-np.pi/3,-np.pi/6,0,np.pi/6,np.pi/3,np.pi/2-.2]
L = [.0799,.4289,.7338,.8011,.7617,.7616,.6299]
U = [.5850,.250,-.1047,-.1047,.1899,.8684,1.133]
R = [2.492,2.250,2.019,np.pi/2,1.277,.6686,.6686]
B = [-1.229,-1.225,-.950,0,.6196,1.362,1.526]
T = [np.pi,np.pi,np.pi,np.pi,np.pi,np.pi,np.pi]


intervals = 16


##########
## Main ##
##########

long_S = Interp(S,intervals)
long_L = Interp(L,intervals)
long_U = Interp(U,intervals)
long_R = Interp(R,intervals)
long_B = Interp(B,intervals)
long_T = Interp(T,intervals)


# Make a real robot 
real_links = np.array([88*np.sqrt(2),400,405,876,50.8]) #mm
links_in = real_links*(1/25.4)
#print(links)
axis = [[0,0,1],[0,1,0],[0,1,0],[0,1,0],[1,0,0]]

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


#Make a bunch of center points with a -5 to 5 degree spread for B and T
theta_start = -5*np.pi/180
theta_end = 5*np.pi/180
d_theta = (theta_end-theta_start)/10 # This number squared is the number of center points created
angle_range = np.arange(theta_start,theta_end,d_theta)
print(angle_range)
anglesDesired = [S[3],L[3],U[3],R[3],B[3],T[3]]


#Changing from normal physics conventions of angle definition to the way the robot defines them:
anglesConvention = [-1*anglesDesired[0],(np.pi/4-anglesDesired[1]),\
                    (-np.pi/2+anglesDesired[2]),-1*anglesDesired[3],-1*anglesDesired[4],np.pi-anglesDesired[5]]

print(anglesConvention)

num = int(len(angle_range))
print(num)
size = num**2
print(size)
matrix_index = 0;
Angles_Matrix = np.zeros((6,size),)

"""
Below is the start of the hill climb. This is a very simple algorithm that randomly
walks around and only takes the improving steps to find a minumum.
This could certainly be improved but it actually works quite well and is reasonably fast.
Creating 100 points takes maybe 10 minutes but then you never have to make them again
This is just order 1 too so 200 points would take 20 minutes etc. 
"""
for i in range(int(len(angle_range))):
    for j in range(int(len(angle_range))):
        
        Target = [End[0],End[1],End[2]] # The end position for B = 0, T = pi, S = 0, R = whatever it is
        angles_start = np.array([anglesConvention[0],anglesConvention[1],\
                           anglesConvention[2],anglesConvention[3],anglesConvention[4]+angle_range[i],\
                                 anglesConvention[5]+angle_range[j]])

        
        angles = angles_start #First Try       
        step = 1e-2 #Step size for the hill searching
        dist = Moto.distanceFromTarget(Target,angles) #How far it is from the home positon at the beginning 


        for k in range(1000): #Hill climb starts

            #print(angles)
            dist = Moto.distanceFromTarget(Target,angles) #Try something
            #print(dist)

            #look somewhere else but don't use all the angles

            # I can only move S, L, and U:
            angles_test = [angles[0]+step*random.uniform(-1,1),angles[1]+step*random.uniform(-1,1),\
                           angles[2]+step*random.uniform(-1,1),angles[3],\
                           angles[4],angles[5]]


            #print(angles_test)
            dist_test = Moto.distanceFromTarget(Target,angles_test) #try that
            #print(dist_test)

            #Was it better?
            if dist_test<=dist:
                angles = angles_test;
                #print('Angles Changed')
            else:
                continue

        #print(angles)
        #print(anglesConvention)
        Angles_Matrix[:,matrix_index] = angles
        matrix_index +=1

Matrix = np.transpose(Angles_Matrix)
print(Matrix[2])


"""
Since I had to switch to angle convention for this script, we have to 
switch back to the desired angles that the robot understands

"""


backToDesired = np.zeros([size,6])
# creating an empty canvas
#fig = plt.figure(figsize = (15,15))

for i in range(size):
    backToDesired[i,:] = [-1*Matrix[i][0],((Matrix[i][1]-np.pi/4)*-1),\
                    (np.pi/2+Matrix[i][2]),-1*Matrix[i][3],-1*Matrix[i][4],(Matrix[i][5]-np.pi)*-1]
    S[3],L[3],U[3],R[3],B[3],T[3] = backToDesired[i,:]
    i+=1
    #Plot them all
    
#     ax = fig.add_subplot(num,num,i+1)
#     ax.plot(index,S,'-o',index,L,'-o',index,U,'-o',index,R,'-o',index,B,'-o',index,T,'-o')
#     #plt.legend(['S','L','U','R','B','T'],fontsize = 12)
#     ax.set_title('Human Swing Joint Space',fontsize = 18)
#     ax.set_xlabel('Waypoint',fontsize = 16)
#     ax.set_ylabel('Radians',fontsize = 16)

#     #print(backToDesired[0])
# plt.show()

#Write the angles to a csv with no headers
import csv
with open('CenterPoints2.csv', 'w', encoding='UTF8', newline='') as f:
    writer = csv.writer(f)
    # write the data
    writer.writerows(backToDesired)