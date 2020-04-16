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


#***Dynamic parameters of DRONE****
L=25 
b = 1e-5 
I = np.diag([5e-7, 5e-7, 10e-7]) 
k=3e-5
m=0.5
kd=0.25
g=9.80


#inicialization of variables
N=300
Thrust, Thrust2 =5, 5

dt=0.1

Kxd1, Kxp1 = 0.76, 0.22
Kyd1, Kyp1 = 0.76, 0.22
Kxd2, Kxp2 = 0.76, 0.22

#x_pos_desired = np.linspace(0,200,N)
final_x_coordinate=150 
x_pos_desired = [final_x_coordinate]*N
x_pos_desired[0:150] = np.linspace(0,final_x_coordinate,150)
#x_pos_desired = np.linspace(0,final_x_coordinate,N)

y_pos_desired = [0]*N
#x_vel_desired = (x_pos_desired - x_pos)/(10*dt)
#x_accel_desired = (x_pos_desired-x_pos)/(100*dt*dt)

#y_pos_desired = 0
#y_vel_desired = (y_pos_desired - y)/(10*dt)
#y_accel_desired = (y_pos_desired-y)/(100*dt*dt)


#State space representation: [theta     phi     gamma 
                            #theta_dot phi_dot gamma_dot 
                            # x         y        z 
                            # x_dot     y_dot   z_dot]
States=  [0]*12
States2= [0]*12
States2[6]=-50 #starting position
#Desired states representation: [x_pos_d, x_vel_d, x_accel_d, 
                                # y_pos_d, y_vel_d, y_accel_d, 
                                # theta_d, phi_d, psi_d, height_desired]
S_desired = [0]*10
S_desired2= [0]*10



for j in range(N):
            
    Old_States=States[5:7] #temporary function
    
    x_vel_desired = (x_pos_desired[j] - States[6])/(50*dt)
    x_accel_desired = (x_vel_desired - States[9])/(100*dt*dt)
    x_accel_desired = 0
    y_vel_desired = (y_pos_desired[j] - States[7])/(50*dt)
    y_accel_desired = (y_vel_desired - States[10])/(100*dt*dt)



    S_desired[0], S_desired[1], S_desired[2] = x_pos_desired[j],x_vel_desired,x_accel_desired
    
    #ANGLE OBJECTIVE
    #Theta
    S_desired[6] = angle_objective(S_desired[0:3],States[6],States[9],States[0],Thrust,Kxp1,Kxd1)
    
    #Phi
    #S_desired[7] = angle_objective(S_desired[3:7],States[7],States[10],States[1],Thrust[j-1],Kyp1,Kyd1)
    if (j>50 and abs(States[6]-final_x_coordinate)<5 and States[9]<0.5 and States[9]>-0.5):
        S_desired[6]=0
        break
    
    #DRONE height position, velocity and acceleration
    #OBTAINING OF THE STATES OF THE QUADROTOR
    States[0:6], Thrust_calc, z_accel, x_accel= quadrotor(States,S_desired[6:11],Thrust)
    
    
    if Thrust_calc-Thrust>0.05:
        Thrust=Thrust_calc+0.05
    elif Thrust-Thrust_calc>0.05:
        Thrust=Thrust_calc-0.05
                

    #OBTAINTION OF FUZZY Kp-Kd
    Kxp1,Kxd1 = OuterLoopAdaptiveController(S_desired[6],States[0],Kxp1,Kxd1) #X Address
    #Kyp,Kyd = (S_desired[6],States[1],Kyp,Kyd) #Y Address

    #OBTAINTION OF X_POS AND X_VEL
    States[9],States[6]= EulerIntegration(x_accel,States[9],States[6])
    

    
    
    ###################################################################
    ###################### Follower DRONE #############################
    ###################################################################
    x_pos_desired2, y_pos_desired2 = Follower(States2[5:7],Old_States)
    x_vel_desired2 = (x_pos_desired2 - States2[6])/(50*dt)
    x_accel_desired2 = 0
    y_vel_desired2 = (y_pos_desired2 - States2[7])/(50*dt)
    y_accel_desired = 0

    S_desired2[0], S_desired2[1], S_desired2[2] = x_pos_desired2,x_vel_desired2,x_accel_desired2
    
    #ANGLE OBJECTIVE
    #Theta
    S_desired2[6] = angle_objective(S_desired2[0:3],States2[6],States2[9],States2[0],Thrust2,Kxp2,Kxd2)
    
    #Phi
    #S_desired[7] = angle_objective(S_desired[3:7],States[7],States[10],States[1],Thrust[j-1],Kyp1,Kyd1)
  
    
    #DRONE height position, velocity and acceleration
    #OBTAINING OF THE STATES OF THE QUADROTOR
    States2[0:6], Thrust_calc2, z_accel2, x_accel2= quadrotor(States2,S_desired2[6:11],Thrust2)
    
    
    if Thrust_calc2-Thrust2>0.05:
        Thrust2=Thrust_calc2+0.05
    elif Thrust2-Thrust_calc2>0.05:
        Thrust2=Thrust_calc2-0.05
                

    #OBTAINTION OF FUZZY Kp-Kd
    Kxp2,Kxd2 = OuterLoopAdaptiveController(S_desired2[6],States2[0],Kxp2,Kxd2) #X Address
    #Kyp,Kyd = (S_desired[6],States[1],Kyp,Kyd) #Y Address

    #OBTAINTION OF X_POS AND X_VEL
    States2[9],States2[6]= EulerIntegration(x_accel2,States2[9],States2[6])



    
    ###################################################################
    ######################## VISUALIZATION ############################
    ###################################################################
    plt.plot(j,x_pos_desired2,'ob')
    #plt.plot(States2[6],'or')
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