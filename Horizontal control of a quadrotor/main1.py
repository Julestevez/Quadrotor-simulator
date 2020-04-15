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


#***Dynamic parameters of DRONE****
L=25 
b = 1e-5 
I = np.diag([5e-7, 5e-7, 10e-7]) 
k=3e-5
m=0.5
kd=0.25
g=9.80


#inicialization of variables
N=200
Torque=[0]*4
Thrust=5
dt=0.1

Kxd1, Kxp1 = 0.76, 0.22
Kyd1, Kyp1 = 0.76, 0.22
Kp, Ki, Kd = 4.05, 0, 20.64
#x_pos_desired = np.linspace(0,200,N) 
x_pos_desired = [100]*N
x_pos_desired[0:180] = np.linspace(0,100,180)
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
States= [0]*12

#Desired states representation: [x_pos_d, x_vel_d, x_accel_d, 
                                # y_pos_d, y_vel_d, y_accel_d, 
                                # theta_d, phi_d, psi_d, height_desired]
S_desired = [0]*10



for j in range(N):
            
    #theta_objective[j]=0.2
    
    x_vel_desired = (x_pos_desired[j] - States[6])/(50*dt)
    x_accel_desired = (x_pos_desired[j] - States[6])/(100*dt*dt)
    y_vel_desired = (y_pos_desired[j] - States[7])/(50*dt)
    y_accel_desired = (y_pos_desired[j] - States[7])/(100*dt*dt)



    S_desired[0], S_desired[1], S_desired[2] = x_pos_desired[j],x_vel_desired,x_accel_desired
    
    #ANGLE OBJECTIVE
    #Theta
    S_desired[6] = angle_objective(S_desired[0:3],States[6],States[9],States[0],Thrust,Kxp1,Kxd1)
    
    #Phi
    #S_desired[7] = angle_objective(S_desired[3:7],States[7],States[10],States[1],Thrust[j-1],Kyp1,Kyd1)
    
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
    #plt.plot(j,States[6],'or')
    plt.plot(j,States[0],'ob')

    #plt.plot(j,States[9],'or') #graphic of x_pos vs time [j = =0.1s]
plt.show()
