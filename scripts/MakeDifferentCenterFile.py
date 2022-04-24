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


"""
The point of this script is to make a bunch of different center points that have
different club angles but the same point of contact. 

"""
index = np.arange(0,7,1)

# NEW SUPER POINTS
S = [-np.pi/2,-np.pi/3,-np.pi/6,0,np.pi/6,np.pi/3,np.pi/2-.2]
L = [.0799,.4289,.7338,.8011,.7617,.7616,.6299]
U = [.5850,.250,-.1047,-.1047,.1899,.8684,1.133]
R = [2.492,2.250,2.019,np.pi/2,1.277,.6686,.6686]
B = [-1.229,-1.225,-.950,0,.6196,1.362,1.526]
T = [np.pi,np.pi,np.pi,np.pi,np.pi,np.pi,np.pi]


intervals = 16


def Interp(theta,intervals):
    long_theta = [] 
    parts = np.zeros((6,intervals-1))
    for i in range(int(len(theta)-1)):
        delta = (theta[i+1]-theta[i])/intervals
        parts[i] = np.linspace(theta[i],theta[i+1]-delta,intervals-1)
    for j in range(int(len(parts))):
        long_theta = np.append(long_theta,parts[j])
    
    return long_theta

long_S = Interp(S,intervals)
long_L = Interp(L,intervals)
long_U = Interp(U,intervals)
long_R = Interp(R,intervals)
long_B = Interp(B,intervals)
long_T = Interp(T,intervals)
                     

class Robot():
    
     
    """
    This is the class for the wire frame robot used in all the scripts.
    This is the way the forward kinematics are calculated for the hill climbs.
    It is also useful for looking at the robot. 
    """
    #Class for the robot with lengths of links and the axis defined as they would be in the defalt zero position
    
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
        v5_0 = [0,self.links[4],0]
        
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
    
    return x,y,z

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

Moto = Robot(links = links_in, axis = axis)
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