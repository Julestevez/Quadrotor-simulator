#main file

#This code represents the control of a single quadrotor in a horizontal motion in X-Y directions

import numpy as np
import math
import matplotlib.pyplot as plt

from mpmath import *
from Quadrotor import quadrotor
from Quadrotor import angle_objective
from Quadrotor import OuterLoopAdaptiveController
from Quadrotor import EulerIntegration
from Visual3D import Visual3D

from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
# This import registers the 3D projection, but is otherwise unused.

#***Dynamic parameters of DRONE****
L=0.25 # [m]
b = 1e-5 
I = np.diag([5e-3, 5e-3, 10e-3])  # [kgm2]
k=3e-5
m=0.5 #[kg]
kd=0.25
g=9.80 #m/s2


#inicialization of variables
N=400 #time steps
Thrust=5
dt=0.1

Kxd1, Kxp1 = 0.76, 0.22
Kyd1, Kyp1 = 0.76, 0.22

#x_pos_desired = np.linspace(0,200,N)
final_x_coordinate=50 
final_y_coordinate=50


#State space representation: [theta     phi     gamma 
                            #theta_dot phi_dot gamma_dot 
                            # x         y        z 
                            # x_dot     y_dot   z_dot]
States= [0]*12

#Desired states representation: [x_pos_d, x_vel_d, x_accel_d, 
                                # y_pos_d, y_vel_d, y_accel_d, 
                                # theta_d, phi_d, psi_d, height_desired]
S_desired = [0]*10

#VISUALIZATION
fig = plt.figure()
ax = fig.gca(projection='3d') 

for j in range(N):
            
    x_vel_desired = (final_x_coordinate - States[6])/(50*dt)
    x_accel_desired = 0
    y_vel_desired = (final_y_coordinate - States[7])/(50*dt)
    y_accel_desired = 0


    S_desired[0], S_desired[1], S_desired[2] = final_x_coordinate,x_vel_desired,x_accel_desired
    S_desired[3], S_desired[4], S_desired[5] = final_y_coordinate,y_vel_desired,y_accel_desired
    
    #DESIRED ANGLE
    #Theta
    S_desired[6] = angle_objective(S_desired[0:3],States[6],States[9],States[0],Thrust,Kxp1,Kxd1)
    
    #Phi
    S_desired[7] = angle_objective(S_desired[3:7],States[7],States[10],States[1],Thrust,Kyp1,Kyd1)
    
    
    #DRONE height position, velocity and acceleration
    #OBTAINING OF THE STATES OF THE QUADROTOR
    States[0:6], Thrust_calc, z_accel, x_accel, y_accel= quadrotor(States,S_desired[6:11],Thrust)
    
    #the increment of the thrust is smooth
    if Thrust_calc-Thrust>0.05:
        Thrust=Thrust_calc+0.05
    elif Thrust-Thrust_calc>0.05:
        Thrust=Thrust_calc-0.05
                

    #OBTAINTION OF X_POS AND X_VEL
    States[9],States[6]  = EulerIntegration(x_accel,States[9],States[6])
    States[10],States[7] = EulerIntegration(y_accel,States[10],States[7])
    
    #########################################################
    ######## VISUALIZATION 1: kinematic variables ###########
    ########################################################
    #plot displacement
    plt.subplot(3,2,1)
    plt.title("Variables in X direction")
    plt.plot(j,States[6],'og',markersize=2)
    plt.ylabel("displacement [m]")

    plt.subplot(3,2,2)
    plt.title("Variables in Y direction")
    plt.plot(j,States[7],'og',markersize=2)
    plt.ylabel("displacement [m]")

    #plot velocity    
    plt.subplot(3,2,3)
    plt.plot(j,States[9],'or',markersize=2)
    plt.ylabel("speed [m/s]")

    #plot velocity    
    plt.subplot(3,2,4)
    plt.plot(j,States[10],'or',markersize=2)
    plt.ylabel("speed [m/s]")
    
    #plot acceleration
    plt.subplot(3,2,5)
    plt.ylabel("acceleration [m/s2]")
    plt.xlabel("time [s/10]")
    plt.plot(j,x_accel,'or',markersize=2)

    plt.subplot(3,2,6)
    plt.ylabel("acceleration [m/s2]")
    plt.xlabel("time [s/10]")
    plt.plot(j,y_accel,'or',markersize=2)

     #########################################################
    #### End of  VISUALIZATION 1: kinematic variables #########
    ########################################################

    
    ###################################################################
    ############### VISUALIZATION 2: 3D Animation ######################
    ###################################################################
    """VecStart_x1,VecStart_y1,VecStart_z1, VecEnd_x1,VecEnd_y1,VecEnd_z1= Visual3D(States[0],States[1],States[6],States[7])
    
  
    ax.cla()
    ax.set_xlim3d(-20, 80)
    ax.set_ylim3d(-20,80)
    ax.set_zlim3d(0,100)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    for i in range(2):
        ax.plot([VecStart_x1[i], VecEnd_x1[i]], [VecStart_y1[i],VecEnd_y1[i]],zs=[VecStart_z1[i],VecEnd_z1[i]])
        
      
    ax.scatter3D(States[6],States[7],50,s=10)
    plt.pause(0.1)"""

    ###################################################################
    ########### End of VISUALIZATION 2: 3D Animation ##################
    ###################################################################
    
        
plt.show()

