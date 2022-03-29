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


# from trajectory_action_client import SimpleTrajectoryActionClient

# import csv
# import os
# import sys
# import rospy
# import actionlib
# import time
# import math

# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
# from sensor_msgs.msg import JointState
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

CenterPoints =[]
cols =6
rows = 100
Matrix = [[0]*cols]*rows
#csv Read
with open('CenterPoint.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)
    next(reader)
    CenterPoints = list(reader)

for i, row in enumerate(CenterPoints):
    for j,val in enumerate(row):
        print(j)
        Matrix[i][j] = float(val)

print(Matrix)
# NEW SUPER POINTS
S = [-np.pi/2,-np.pi/3,-np.pi/6,0,np.pi/6,np.pi/3,np.pi/2-.2]
L = [.0799,.4289,.7338,.8011,.7617,.7616,.6299]
U = [.5850,.250,-.1047,-.1047,.1899,.8684,1.133]
R = [2.492,2.250,2.019,np.pi/2,1.277,.6686,.6686]
B = [-1.229,-1.225,-.950,0,.6196,1.362,1.526]
T = [np.pi,np.pi,np.pi,np.pi,np.pi,np.pi,np.pi]

#Start While with user input

#S[3],L[3],U[3],R[3],B[3],T[3] = 