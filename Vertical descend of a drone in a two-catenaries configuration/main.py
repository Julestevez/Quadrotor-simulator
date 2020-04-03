#This code gets a equiload situation of equilibrium, by the descend of
# a quadrtor to the equilibrium point between two catenaries.

import numpy as np
import math
import matplotlib.pyplot as plt

from mpmath import *
from quadrotor import quadrotor


#***Dynamic parameters of DRONE****
L=25 
b = 1e-5 
I = np.diag([5e-7, 5e-7, 10e-7]) 
k=3e-5
m=0.5
kd=0.25


#inicialization of variables
N=500
z, z_vel, z_acel= [0]*N, [0]*N, [0]*N
phi_acel, theta_acel, psi_acel= [0]*N, [0]*N, [0]*N
phi_vel, theta_vel, psi_vel= [0]*N, [0]*N, [0]*N
phi = theta = psi = [0]*N
Torque=[0]*4
Thrust=10

Height_objective=-10
dt=0.1

     
for i in range(N):
    #DRONE height position, velocity and acceleration

    phi[i], theta[i], psi[i], phi_vel[i], theta_vel[i], psi_vel[i], Thrust, z_acel[i],error= quadrotor(phi[i-1], theta[i-1],psi[i-1],phi_vel[i-1], theta_vel[i-1], psi_vel[i-1], Height_objective, Thrust,z_vel[i-1],z[i-1],z_vel[i-2])
    
    z_vel[i]= z_vel[i-1] + z_acel[i]/2*dt
    
    z[i]= z[i-1] + z_vel[i]/2*dt
    
    plt.plot(i,z[i],'or') #plot of the drone height vs time
    plt.pause(0.1)
plt.show()