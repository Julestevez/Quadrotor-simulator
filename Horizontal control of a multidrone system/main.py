#main file

#This code represents the control of two quadrotors in a horizontal motion in X-Y directions

import numpy as np
import math
import matplotlib.pyplot as plt

from mpmath import *
from Quadrotor import quadrotor
from Quadrotor import angle_objective
from Quadrotor import EulerIntegration
from Quadrotor import Follower
from Quadrotor import VelAccelDesired
from Visual3D import Visual3D


#***Dynamic parameters of DRONE****
L=0.25 #[m]
b = 1e-5 
I = np.diag([5e-3, 5e-3, 10e-3])  #[kgm2]
k=3e-5
m=0.5 #[kg]
kd=0.25
g=9.80 #[m/s2]
dt=0.1 #[s]

###############################################################
############## Inicialization of variables ####################
###############################################################
N=300
Thrust1, Thrust2 =5, 5

Kxd1, Kxp1 = 0.76, 0.22
Kyd1, Kyp1 = 0.76, 0.22
Kxd2, Kxp2 = 0.76, 0.22
Kyd2, Kyp2 = 0.76, 0.22



#FINAL COORDINATES TO REACH
final_x_coordinate=40
final_y_coordinate=30 



#State space representation: [theta     phi     gamma 
                            #theta_dot phi_dot gamma_dot 
                            # x         y        z 
                            # x_dot     y_dot   z_dot]

States1=  [0]*12 #States of the Quadrotor1
States2=  [0]*12 #States of the Quadrotor2

States2[6]=-50 #starting position
States2[7]=-60
#Desired states representation: [x_pos_d, x_vel_d, x_accel_d, 
                                # y_pos_d, y_vel_d, y_accel_d, 
                                # theta_d, phi_d, psi_d, height_desired]
S_desired1 = [0]*10
S_desired2 = [0]*10

###############################################################
################# end of initialization ######################
###############################################################

#VISUALIZATION
fig = plt.figure() 
#ax = fig.gca(projection='3d') #this line is only useful for 3d projection, not for ploting graphs


for j in range(N):
            
    Old_States=States1[5:7] #temporary variables
    
    #Calculus of DESIRED vel and accel
    x_vel_desired1,x_accel_desired1 = VelAccelDesired(final_x_coordinate,States1[6])
    y_vel_desired1,y_accel_desired1 = VelAccelDesired(final_y_coordinate,States1[7])
    
    #Assign desired position, speed and velocity in X axis
    S_desired1[0], S_desired1[1], S_desired1[2] = final_x_coordinate,x_vel_desired1,x_accel_desired1

    #Assign desired position, speed and velocity in Y axis
    S_desired1[3], S_desired1[4], S_desired1[5] = final_y_coordinate,y_vel_desired1,y_accel_desired1
    
    #OBJECTIVE ANGLES
    #Theta
    S_desired1[6] = angle_objective(S_desired1[0:3],States1[6],States1[9],States1[0],Thrust1,Kxp1,Kxd1)
    
    #Phi
    S_desired1[7] = angle_objective(S_desired1[3:7],States1[7],States1[10],States1[1],Thrust1,Kyp1,Kyd1)
    
    
    #DRONE height position, velocity and acceleration
    #OBTAINING OF THE STATES OF THE QUADROTOR
    States1[0:6], Thrust_calc1, z_accel1, x_accel1, y_accel1= quadrotor(States1,S_desired1[6:11],Thrust1)
    
    
    #smoothing of the thrust
    if Thrust_calc1-Thrust1>0.05:
        Thrust1=Thrust_calc1+0.05
    elif Thrust1-Thrust_calc1>0.05:
        Thrust1=Thrust_calc1-0.05
                

    #OBTAINING OF POS AND VELOCITY
    States1[9],States1[6]= EulerIntegration(x_accel1,States1[9],States1[6]) #X axis
    States1[10],States1[7]= EulerIntegration(y_accel1,States1[10],States1[7]) #y axis
    

    
    ###################################################################
    ###################### Follower DRONE #############################
    ###################################################################
    x_pos_desired2, y_pos_desired2 = Follower(States1[6:8],Old_States) #previous drone states as argument
    
    #Calculus of DESIRED vel and accel
    x_vel_desired2,x_accel_desired2 = VelAccelDesired(x_pos_desired2,States2[6])
    y_vel_desired2,y_accel_desired2 = VelAccelDesired(y_pos_desired2,States2[7])
    
    #Assign desired position, speed and velocity in X axis
    S_desired2[0], S_desired2[1], S_desired2[2] = x_pos_desired2,x_vel_desired2,x_accel_desired2

    #Assign desired position, speed and velocity in Y axis
    S_desired2[3], S_desired2[4], S_desired2[5] = y_pos_desired2,y_vel_desired2,y_accel_desired2
    
    #ANGLE OBJECTIVE 
    #Theta - X direction
    S_desired2[6] = angle_objective(S_desired2[0:3],States2[6],States2[9],States2[0],Thrust2,Kxp2,Kxd2)
    
    #Phi - Y direction
    S_desired2[7] = angle_objective(S_desired2[3:7],States2[7],States2[10],States2[1],Thrust2,Kyp2,Kyd2)
    
    
    #DRONE height position, velocity and acceleration
    #OBTAINING OF THE STATES OF THE QUADROTOR
    States2[0:6], Thrust_calc2, z_accel2, x_accel2, y_accel2= quadrotor(States2,S_desired2[6:11],Thrust2)
    
    #smoothing of the thrust
    if Thrust_calc2-Thrust2>0.05:
        Thrust2=Thrust_calc2+0.05
    elif Thrust2-Thrust_calc2>0.05:
        Thrust2=Thrust_calc2-0.05
                

    #OBTAINTION OF POS and VELOCITY
    States2[9],States2[6]= EulerIntegration(x_accel2,States2[9],States2[6]) #x axis
    States2[10],States2[7]= EulerIntegration(y_accel2,States2[10],States2[7]) #y axis

   
    #########################################################
    ########## VISUALIZATION 1: 3d projection## ############
    ########################################################
    VecStart_x1,VecStart_y1,VecStart_z1, VecEnd_x1,VecEnd_y1,VecEnd_z1= Visual3D(States1[0],States1[1],States1[6],States1[7])
    VecStart_x2,VecStart_y2,VecStart_z2, VecEnd_x2,VecEnd_y2,VecEnd_z2= Visual3D(States2[0],States2[1],States2[6],States2[7])
  

    """ax.cla()
    ax.set_xlim3d(-100, 200)
    ax.set_ylim3d(-100,100)
    ax.set_zlim3d(0,100)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    for i in range(2):
        ax.plot([VecStart_x1[i], VecEnd_x1[i]], [VecStart_y1[i],VecEnd_y1[i]],zs=[VecStart_z1[i],VecEnd_z1[i]])
        ax.plot([VecStart_x2[i], VecEnd_x2[i]], [VecStart_y2[i],VecEnd_y2[i]],zs=[VecStart_z2[i],VecEnd_z2[i]])
        
    
    ax.scatter3D(States1[6],States1[7],50,s=30)
    ax.scatter3D(States2[6],States2[7],50,s=30)
    #ax.scatter3D(States3[6],States3[7],0)
    #ax.scatter3D(vectorX1,0,z1)"""

    #########################################################
    ###### End of  VISUALIZATION 1: 3d projection ##########
    ########################################################


    ###################################################################
    ######### VISUALIZATION 2: X-Y displacement of 2 drones ###########
    ###################################################################

    #plot displacement
    plt.subplot(2,2,1)
    plt.title("Variables of Drone1")
    plt.plot(j,States1[6],'og',markersize=2)
    plt.ylabel("displacement in X [m]")

    plt.subplot(2,2,2)
    plt.title("Variables of Drone2")
    plt.plot(j,States2[6],'og',markersize=2)
    plt.ylabel("displacement in X [m]")

    plt.subplot(2,2,3)
    plt.plot(j,States1[7],'or',markersize=2)
    plt.ylabel("displacement in Y [m]")

    plt.subplot(2,2,4)
    plt.plot(j,States2[7],'or',markersize=2)
    plt.ylabel("displacement in Y [m]")


    ###################################################################
    ####### End of VISUALIZATION 2: X-Y displacement of 2 drones ######
    ###################################################################
plt.show()
