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
from Quadrotor import Follower
from Quadrotor import VelAccelDesired
from Visual3D import Visual3D


#***Dynamic parameters of DRONE****
L=25 
b = 1e-5 
I = np.diag([5e-7, 5e-7, 10e-7]) 
k=3e-5
m=0.5
kd=0.25
g=9.80

###############################################################
##############inicialization of variables######################
###############################################################
N=300
Thrust1, Thrust2 =5, 5

Kxd1, Kxp1 = 0.76, 0.22
Kyd1, Kyp1 = 0.76, 0.22
Kxd2, Kxp2 = 0.76, 0.22
Kyd2, Kyp2 = 0.76, 0.22

dt=0.1


#FINAL COORDINATES TO REACH
final_x_coordinate=150
final_y_coordinate=80 

#DISCRETIZED VARIABLES OF DESIRED VALUES FOR DRONE_LEADER
x_pos_desired1 = [final_x_coordinate]*N
x_pos_desired1[0:150] = np.linspace(0,final_x_coordinate,150)

y_pos_desired1 = [final_y_coordinate]*N
y_pos_desired1[0:150] = np.linspace(0,final_y_coordinate,150)


#State space representation: [theta     phi     gamma 
                            #theta_dot phi_dot gamma_dot 
                            # x         y        z 
                            # x_dot     y_dot   z_dot]

States1=  [0]*12 #States of the Quaddrotor1
States2=  [0]*12 #States of the Quadrotor2

States2[6]=-50 #starting position
States2[7]=-60
#Desired states representation: [x_pos_d, x_vel_d, x_accel_d, 
                                # y_pos_d, y_vel_d, y_accel_d, 
                                # theta_d, phi_d, psi_d, height_desired]
S_desired1 = [0]*10
S_desired2= [0]*10

###############################################################
################# end of initialization ######################
###############################################################

fig = plt.figure()
ax = fig.gca(projection='3d') 
for j in range(N):
            
    Old_States=States1[5:7] #temporary function
    
    #Calculus of DESIRED vel and accel
    x_vel_desired1,x_accel_desired1 = VelAccelDesired(x_pos_desired1[j],States1[6])
    y_vel_desired1,y_accel_desired1 = VelAccelDesired(y_pos_desired1[j],States1[7])
    

    #Assign desired position, speed and velocity in X axis
    S_desired1[0], S_desired1[1], S_desired1[2] = x_pos_desired1[j],x_vel_desired1,x_accel_desired1

    #Assign desired position, speed and velocity in Y axis
    S_desired1[3], S_desired1[4], S_desired1[5] = y_pos_desired1[j],y_vel_desired1,y_accel_desired1
    
    #OBJECTIVE ANGLES
    #Theta
    S_desired1[6] = angle_objective(S_desired1[0:3],States1[6],States1[9],States1[0],Thrust1,Kxp1,Kxd1)
    
    #Phi
    S_desired1[7] = angle_objective(S_desired1[3:7],States1[7],States1[10],States1[1],Thrust1,Kyp1,Kyd1)
    if (j>50 and abs(States1[6]-final_x_coordinate)<5 and States1[9]<0.5 and States1[9]>-0.5):
        S_desired1[6]=0
        break
    
    #DRONE height position, velocity and acceleration
    #OBTAINING OF THE STATES OF THE QUADROTOR
    States1[0:6], Thrust_calc1, z_accel1, x_accel1, y_accel1= quadrotor(States1,S_desired1[6:11],Thrust1)
    
    
    if Thrust_calc1-Thrust1>0.05:
        Thrust1=Thrust_calc1+0.05
    elif Thrust1-Thrust_calc1>0.05:
        Thrust1=Thrust_calc1-0.05
                

    #OBTAINING OF FUZZY Kp-Kd
    Kxp1,Kxd1 = OuterLoopAdaptiveController(S_desired1[6],States1[0],Kxp1,Kxd1) #X Address
    Kyp1,Kyd1 = OuterLoopAdaptiveController(S_desired1[7],States1[1],Kyp1,Kyd1) #Y Address

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
    if (j>50 and abs(States2[6]-S_desired2[3])<5 and States2[10]<0.5 and States2[10]>-0.5):
        S_desired2[7]=0
        break
    
    #DRONE height position, velocity and acceleration
    #OBTAINING OF THE STATES OF THE QUADROTOR
    States2[0:6], Thrust_calc2, z_accel2, x_accel2, y_accel2= quadrotor(States2,S_desired2[6:11],Thrust2)
    
    
    if Thrust_calc2-Thrust2>0.05:
        Thrust2=Thrust_calc2+0.05
    elif Thrust2-Thrust_calc2>0.05:
        Thrust2=Thrust_calc2-0.05
                

    #OBTAINTION OF FUZZY Kp-Kd
    Kxp2,Kxd2 = OuterLoopAdaptiveController(S_desired2[6],States2[0],Kxp2,Kxd2) #X Address
    Kyp2,Kyd2 = OuterLoopAdaptiveController(S_desired2[7],States2[1],Kyp2,Kyd2) #Y Address

    #OBTAINTION OF POS and VELOCITY
    States2[9],States2[6]= EulerIntegration(x_accel2,States2[9],States2[6]) #x axis
    States2[10],States2[7]= EulerIntegration(y_accel2,States2[10],States2[7]) #y axis

   
    ###################################################################
    ######################## VISUALIZATION ############################
    ###################################################################
    VecStart_x1,VecStart_y1,VecStart_z1, VecEnd_x1,VecEnd_y1,VecEnd_z1= Visual3D(States1[0],States1[1],States1[6],States1[7])
    VecStart_x2,VecStart_y2,VecStart_z2, VecEnd_x2,VecEnd_y2,VecEnd_z2= Visual3D(States2[0],States2[1],States2[6],States2[7])
  

    ax.cla()
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
    #ax.scatter3D(vectorX1,0,z1)
   

    #plt.plot(j,States[6],'ob')          #plot of leader drone
    #plt.plot(j,x_pos_desired2,'ob')    #plot of desired position of follower drone
    #plt.plot(j,States2[6],'or')        #plot of follower drone

    #plt.gca()

    #plt.plot(j,States[9],'or') #graphic of x_pos vs time [j = =0.1s]

    #plt.plot(States[6],States[7],'ob')
    #plt.plot(States2[6],States2[7],'or')
    #plt.plot(j,States2[6],'or')
    #plt.plot(j,y_pos_desired2,'*g')
    #plt.plot(j,y_pos_desired2,'or')
    #plt.plot(j,States2[6],'or')
    plt.pause(0.1)

    #plt.plot(j,States[9],'or') #graphic of x_pos vs time [j = =0.1s]
plt.show()

"""
    plt.subplot(3, 1, 1)
    plt.plot(j,x_accel[j],'or') #plot of the drone height vs time
    plt.subplot(3,1,2)
    plt.plot(j,x_vel[j],'ob')
    plt.subplot(3,1,3)
    plt.plot(j,x_pos[j],'og')
    plt.pause(0.1)
plt.show()"""
