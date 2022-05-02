import numpy as np
import scipy.integrate as integrate
from scipy.integrate import odeint, solve_ivp
import csv

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








class Robot():
    
    
    
    """
    This is the class for the wire frame robot used in all the scripts.
    This is the way the forward kinematics are calculated for the hill climbs.
    It is also useful for looking at the robot. 
    """
    
    def __init__(self, links, axis = [[0,0,1],[0,1,0],[0,1,0],[0,1,0],[1,0,0]]):
        self.links = links
        self.axis = axis
        
    def rotateAxis(self, t, vec):
        
        #Rotation matrix just taken straight from wikepedia
        
        C = 1-np.cos(t)
        x,y,z = vec

        Rot = [[x**2*C+np.cos(t), x*y*C-z*np.sin(t), z*x*C+y*np.sin(t)],
                [x*y*C+z*np.sin(t), y**2*C+np.cos(t), z*y*C-x*np.sin(t)],
                [x*z*C-y*np. sin(t), y*z*C+x*np.sin(t), z**2*C+np.cos(t)]]

        return Rot

    def findEnd(self, angles):
        #This function takes an input vector of angles and will output the end position as
        #well as all of the vectors of the robot NOT TRANSLATED FROM THE ORIGIN
        
        v1 = [0,0,0] #Base thing that moves with S
        v2 = [0,0,0] #First link that moves with L
        v3 = [0,0,0] #Second link that moves with U
        v4 = [0,0,0] #Shaft link that moves with B
        v5 = [0,0,0] #Club head that moves with T/R
        
        #Starting Vector
        v1_0 = [self.links[0]/np.sqrt(2),0,self.links[0]/np.sqrt(2)]
        v5_0 = [self.links[4]*np.cos(np.pi/6),self.links[4]*np.cos(np.pi/12),self.links[4]*np.cos(np.pi/3)]
        
        #First rotation
        v1 = np.dot(v1_0,self.rotateAxis(angles[0], self.axis[0]))
        axis1 = np.dot(self.axis[1],self.rotateAxis(angles[0], self.axis[0]))
        axis2 = np.dot(self.axis[2],self.rotateAxis(angles[0], self.axis[0]))
        axis3 = np.dot(self.axis[3],self.rotateAxis(angles[0], self.axis[0]))
        axis4 = np.dot(self.axis[4],self.rotateAxis(angles[0], self.axis[0]))
        
        #Second Rotation
        v2 = (self.links[1]/self.links[0])*np.dot(v1,self.rotateAxis(angles[1], axis1))
        axis2 = np.dot(axis2,self.rotateAxis(angles[1], axis1))
        axis3 = np.dot(axis3,self.rotateAxis(angles[1], axis1))
        axis4 = np.dot(axis4,self.rotateAxis(angles[1], axis1))
        
        #Third Rotation
        v3 = (self.links[2]/self.links[1])*np.dot(v2,self.rotateAxis(angles[2], axis2))
        axis3 = np.dot(axis3,self.rotateAxis(angles[2], axis2))
        axis4 = np.dot(axis4,self.rotateAxis(angles[2], axis2))

        #Fourth rotation
        axis3 = np.dot(axis3,self.rotateAxis(angles[3], v3/self.links[2])) # Can only rotate around unit vectors
        axis4 = np.dot(axis4,self.rotateAxis(angles[3], v3/self.links[2]))
        
        #Fifth Rotation
        v4 = (self.links[3]/self.links[2])*np.dot(v3,self.rotateAxis(angles[4], axis3))
        axis4 = np.dot(axis4,self.rotateAxis(angles[4], axis3))
        
        #Sixth Rotation
        v5 = np.dot(v5_0,self.rotateAxis(angles[5], axis4))
        
        
        x_val = v1[0]+v2[0]+v3[0]+v4[0]+v5[0]
        y_val = v1[1]+v2[1]+v3[1]+v4[1]+v5[1]
        z_val = v1[2]+v2[2]+v3[2]+v4[2]+v5[2]
        
        return [x_val, y_val, z_val, v1, v2, v3, v4, v5]
    
    def distanceFromTarget(self,targ,angles):
        
        #Simply finds the distance between a given end from a set of angles and a target end point
        
        end = self.findEnd(angles)
        dist = (end[0]-targ[0])**2+(end[1]-targ[1])**2+(end[2]-targ[2])**2
        
        return dist
    

def drawRobot2(v1,v2,v3,v4,v5):
    
    """
    The purpose of this function is just to convert the five vectors into
    (x,y,z) arrays for plotting. 
    
    If you ever see drawRobot, that one is old so delete that
    """
    x = np.zeros(6,)
    y = np.zeros(6,)
    z = np.zeros(6,)
    
    x[0],y[0],z[0] = [0,0,0]
    x[1],y[1],z[1] = v1
    x[2],y[2],z[2] = v1+v2
    x[3],y[3],z[3] = v1+v2+v3
    x[4],y[4],z[4] = v1+v2+v3+v4
    x[5],y[5],z[5] = v1+v2+v3+v4+v5
    
    # print(x)
    # print(y)
    # print(z)
    
    return x,y,z # The arrays are all just 1x6





def makeTime(x_end,y_end,z_end,max_vel):
    
    """
    The way velocity selection works is by dividing the distance betweeen
    two (x,y,z) way points by the desired velocity then setting the time step 
    between the two points to that value. In this way the times for a swing are 
    calculated rather than specified.
    """
    #Initalize Everything
    pts = len(x_end)
    time = 0
    t_array = np.array([0])
    vel = np.zeros(pts,)
    dist = np.zeros(pts-1,)
    
    #Make the velocity of the actual club a gauss curve
    for i in range(pts-1):
        sigma = 20
        vel[i] = max_vel*np.exp(-(i-(pts/2))**2/(2*sigma**2))
        
    for i in range(pts-1):
        """
        loops through all the small ds along the curve and add the next 
        time based on how fast that section should be.
        """
        dist[i] = np.sqrt((x_end[i+1]-x_end[i])**2+(y_end[i+1]-y_end[i])**2+(z_end[i+1]-z_end[i])**2)
        time = time + dist[i]/vel[i]
        t_array = np.append(t_array,time)
        
    return t_array,vel,dist


"""
The next section of the code is for reading in all the different center points
then replacing the normal center point and swinging. 
"""
CenterPoints =[]
cols = 6
rows = 100
Matrix = []
#csv Read
with open('CenterPoints.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)
    CenterPoints = list(reader)
for i, row in enumerate(CenterPoints):
    center_pos = []
    for j,val in enumerate(row):
        center_pos.append(float(val))
    Matrix.append(center_pos)

# print(Matrix)
# NEW SUPER POINTS
S = [-np.pi/2,-np.pi/3,-np.pi/6,0,np.pi/6,np.pi/3,np.pi/2-.2]
L = [.0799,.4289,.7338,.8011,.7617,.7616,.6299]
U = [.5850,.250,-.1047,-.1047,.1899,.8684,1.133]
R = [2.492,2.250,2.019,np.pi/2,1.277,.6686,.6686]
B = [-1.229,-1.225,-.950,0,.6196,1.362,1.526]
T = [np.pi,np.pi,np.pi,np.pi,np.pi,np.pi,np.pi]

# This just turns the 7 points into 90 points linearly spaced in joint space
def Interp(theta,intervals):
    long_theta = [] 
    parts = np.zeros((6,intervals-1))
    for i in range(int(len(theta)-1)):
        delta = (theta[i+1]-theta[i])/intervals
        parts[i] = np.linspace(theta[i],theta[i+1]-delta,intervals-1)
    for j in range(int(len(parts))):
        long_theta = np.append(long_theta,parts[j])
    
    return long_theta



#Start While with user input
idx = 0
err = 0
while(1):
    print("Last index used: " + str(idx+1))
    print(Matrix[idx])
    increment = '2'
    uResponse = input("Press enter to increment by " + increment + " from the previous index, or enter a new index (ctrl Z to exit): ")
    if uResponse == "":
        idx+=int(increment)
    elif uResponse.isnumeric():
        if int(uResponse) < len(Matrix):
            idx= int(uResponse)-1
        else:
            print("\n\nERROR: Input was either not a number or indexed out of the bounds of the CSV file. Please try a new index...\n")
            err = 1
    else:
        print("\n\nERROR: Input was either not a number or indexed out of the bounds of the CSV file. Please try a new index...\n")
        err = 1


    if err != 1:
        
        #Replace the normal super point center with one from the matrix of new center points

        S[3],L[3],U[3],R[3],B[3],T[3] = Matrix[idx][0],Matrix[idx][1],Matrix[idx][2],Matrix[idx][3],Matrix[idx][4],Matrix[idx][5]
        print(S[3],L[3],U[3],R[3],B[3],T[3])
        intervals = 16
        
        #Look at them to check 
        long_S = Interp(S,intervals)
        long_L = Interp(L,intervals)
        long_U = Interp(U,intervals)
        long_R = Interp(R,intervals)
        long_B = Interp(B,intervals)
        long_T = Interp(T,intervals)
                            
        pts = len(long_R)
        print(len(long_R))
        print(long_R)
        
        
        # Make a real robot 
        real_links = np.array([5.464,20.97,20.97,34.5,2]) #Inches
        links = real_links*(1/20) #Scaling it down I guess not actually sure what the point of this is but okay
        #print(links)
        axis = [[0,0,1],[0,1,0],[0,1,0],[0,1,0],[1,0,0]]
        anglesDesired = [S[3],L[3],U[3],R[3],B[3]-np.pi/4,T[3]]

        #Changing from normal physics conventions of angle definition to the way the robot defines them:
        anglesConvention = [-1*anglesDesired[0],(np.pi/4-anglesDesired[1]),\
                            (-np.pi/2+anglesDesired[2]),anglesDesired[3],anglesDesired[4],np.pi-anglesDesired[5]]
            
        #Calling and instance of the robot class
        Moto = Robot(links = real_links, axis = axis)
        End = Moto.findEnd(anglesConvention)
        print([End[0],End[1],End[2]])
        v1,v2,v3,v4,v5 = End[3],End[4],End[5],End[6],End[7]
        x,y,z = drawRobot2(v1,v2,v3,v4,v5)

        

        # Go from joint space to (x,y,z) space to define actual club head velocity
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
        
        #Set the max velocity at conact in ft/sec*12 inches
        max_vel = 35*12 #in/sec
        t_array,vel,dist = makeTime(x_end,y_end,z_end,max_vel)

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

        index = np.arange(0,7,1)
        # fig4 = plt.figure(figsize = (14,7))
        # ax4 = fig4.add_subplot(1,2,1)
        # ax5 = fig4.add_subplot(1,2,2)
        # ax4.plot(t_array,long_S,'-')
        # ax4.plot(t_array,long_L,'-')
        # ax4.plot(t_array,long_U,'-')
        # ax4.plot(t_array,long_R,'-')
        # ax4.plot(t_array,long_B,'-')
        # ax4.plot(t_array,long_T,'-')
        # ax5.plot(index,S,'-o')
        # ax5.plot(index,L,'-o')
        # ax5.plot(index,U,'-o')
        # ax5.plot(index,R,'-o')
        # ax5.plot(index,B,'-o')
        # ax5.plot(index,T,'-o')
        # plt.legend(['S','L','U','R','B','T'],fontsize = 16)
        # plt.show()
        print('Printing vel_s')
        print(vel_s)

        """
        Here is where the actual swinging starts and the communication with the robot.
        The way the commands work is sending position and velocity goals at set times.
        Everything in the middle is determined at some lower level where I assume some
        kind of boundary condition calculations are happening. If you send enough commands
        over a short amount of time, you can come close to complete control of the robot. 
        
        """
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
        #Do the forward swing
        for i in range(len(t_array)):
            traj_plan_swing.add_joint_waypoint([long_S[i],long_L[i],long_U[i],long_R[i],long_B[i],long_T[i]],t_array[i]+t_start,[vel_s[i],vel_l[i],vel_u[i],vel_r[i],vel_b[i],vel_t[i]])
        # Swing back too the middle on the same path but slower
        for j in range(int(len(t_array)/2)-1):
            j+=1
            traj_plan_swing.add_joint_waypoint([long_S[-j],long_L[-j],long_U[-j],long_R[-j],long_B[-j],long_T[-j]],t_array[-1]+t_start+t_back[j],[0,0,0,0,0,0])
        #Go a little further than middle to get past the tee
        traj_plan_swing.add_joint_waypoint([long_S[40],long_L[40],long_U[40],long_R[40],long_B[40],long_T[40]],t_array[-1]+t_start+t_back[j]+2,[0,0,0,0,0,0])

        traj_plan_swing.send_trajectory()
    err = 0