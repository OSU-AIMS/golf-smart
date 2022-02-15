#!/usr/bin/env python
# coding: utf-8

# In[1]:



# In[2]:


import numpy as np
import scipy.integrate as integrate
from scipy.integrate import odeint, solve_ivp



import matplotlib.pyplot as plt
from matplotlib import animation, rc
from mpl_toolkits import mplot3d
import random as random
from mpl_toolkits.mplot3d import Axes3D
from IPython.display import HTML



#New Stuff!!!!!

#I want to make a new function that I'm gonna call swing

def swing(amp,flatness,width,center,t_pts):
    for i in range(len((t_pts))):
        vel[i] = amp/(1+abs((t_pts[i]-center)/width)**(flatness))
    return vel


# In[97]:


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


# In[98]:


def sig_int(amp,width):
    integral = amp*width*2
    return integral


# In[99]:


def find_width(amp,delta_theta):
    width = delta_theta/(amp*2)
    return width


# In[109]:


t_start = 0
t_end = 2
delta_t = (t_end-t_start)/100;
theta_start = -np.pi/2
theta_end = np.pi/2
delta_theta = theta_end-theta_start
t_pts = np.arange(t_start,t_end,delta_t);

# Hips move Gauss S
S_start = -np.pi*2/3;
S_end = np.pi*2/3;
S_delta_theta = (S_end-S_start)
center_s = .8
amp_s = 4.3
width_s = find_width(amp_s,S_delta_theta)
vel_s = sigmoid(amp_s,width_s,center_s,t_pts)
S = S_start + num_integrate(vel_s,t_pts)



# Wrist Moves Gauss B
B_start = np.pi*2/3
B_end = -np.pi*2/3
B_delta_theta = (B_end-B_start)*-1
center_b = 1
amp_b = 15
width_b = find_width(amp_b,B_delta_theta)
vel_b = -1*sigmoid(amp_b,width_b,center_b,t_pts)
B = B_start + num_integrate(vel_b,t_pts)


L = np.ones(len(t_pts))*np.pi/24;
U = np.ones(len(t_pts))*-np.pi/3;
R = np.ones(len(t_pts))*np.pi/2;

