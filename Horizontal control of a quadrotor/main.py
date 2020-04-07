#main file

#This code gets a equiload situation of equilibrium, by the descend of
# a quadrtor to the equilibrium point between two catenaries.

import numpy as np
import math
import matplotlib.pyplot as plt

from mpmath import *
from Quadrotor import quadrotor


#***Dynamic parameters of DRONE****
L=25 
b = 1e-5 
I = np.diag([5e-7, 5e-7, 10e-7]) 
k=3e-5
m=0.5
kd=0.25
g=9.80


#inicialization of variables
N=500
x_pos, x_vel, x_accel= [0]*N, [0]*N, [0]*N
y_pos, y_vel, y_accel= [0]*N, [0]*N, [0]*N
z_pos, z_vel, z_accel= [0]*N, [0]*N, [0]*N

phi_acel, theta_acel, psi_acel= [0]*N, [0]*N, [0]*N
phi_vel, theta_vel, psi_vel= [0]*N, [0]*N, [0]*N
phi, theta, psi= [0]*N, [0]*N, [0]*N
Torque=[0]*4
Thrust=[5]*N
theta_objective=[0]*N


dt=0.1

Kxd1, Kxp1 = 0.76, 0.22
Kyd1, Kyp1 = 0.76, 0.22
Kp, Ki, Kd = 4.05, 0, 20.64
x_pos_desired = 50
#x_vel_desired = (x_pos_desired - x_pos)/(10*dt)
#x_accel_desired = (x_pos_desired-x_pos)/(100*dt*dt)

#y_pos_desired = 0
#y_vel_desired = (y_pos_desired - y)/(10*dt)
#y_accel_desired = (y_pos_desired-y)/(100*dt*dt)

Ux_dcha=[]*N
a=[]*N

Kd1, Kp1=1, 0.8


for j in range(N):
  
    #theta_objective[j]=0.2
    
    #x_vel_desired[j] = (x_pos_desired - x_pos[j])/(10*dt)
    x_vel_desired = (x_pos_desired - x_pos[j-1])/(50*dt)
    #x_accel_desired[j] = (x_pos_desired-x_pos[j])/(100*dt*dt)
    x_accel_desired = (x_pos_desired-x_pos[j-1])/(100*dt*dt)



    Ux_dcha=x_accel_desired + Kxd1*(x_vel_desired - x_vel[j-1]) + Kxp1*(x_pos_desired-x_pos[j-1])
    a=Ux_dcha*m/(Thrust[j-1]*math.cos(theta[j-1]))
    #chapuza
    if a>0.7:
        a=0.7
    elif a<-0.7:
        a=-0.7
        
    theta_objective[j]=math.asin(a)


    """Uy_dcha[j]=y_acel_deseado + Kd1*(y_vel_deseado - y_vel_dcha[j-1]) + Kp1*(y_deseado-y_pos_dcha[j-1])
            
    a_[j]=Uy_dcha[j]*m/(Thrust[j-1])
            
    if a_[j]>0.5:
        a_[j]=0.5
    elif a_[j]<-0.5:
        a_[j]=-0.5
            
    phi_objetivo[j]=-math.asin(a_[j])"""
    Height_objective=0

    #DRONE height position, velocity and acceleration

    #phi[j], theta[j], psi[j], phi_vel[j], theta_vel[j], psi_vel[j], Thrust[j], z_accel[j],error= quadrotor(phi[j-1], theta[j-1],psi[j-1],phi_vel[j-1], theta_vel[j-1], psi_vel[j-1], Height_objective, Thrust,z_vel[j-1],z_pos[j-1],z_vel[j-2],theta_objective[j])
    err_theta=theta_objective[j]-theta[j-1]
    Torque[2]=(err_theta*Kp + err_theta*Ki*dt - theta_vel[j-1]*Kd*dt) * I[1,1]
    err_height=0
    Thrust[j]=err_height*Kp + err_height*Ki*dt - (z_vel[j-1]-z_vel[j-2])*Kd*dt + (m*g)/(math.cos(phi[j-1])*math.cos(theta[j-1])) 
    if Thrust[j]>10:
        Thrust[j]=10
    elif Thrust[j]<3:
        Thrust[j]=3
    
    theta_acel=  Torque[2]/I[1,1] 
      
    theta_vel[j]= theta_vel[j-1] + theta_acel*dt/2
       
    theta[j]=     theta[j-1] + theta_vel[j]*dt/2 


    P_ex= (theta_objective[j]-theta[j])/(theta_objective[j])*100
    if (abs(P_ex)> 1 and abs(P_ex)<=3):
        mu=0.5*P_ex-0.5
        Kxd1=abs(Kxd1 + 0.5*mu*(theta_objective[j]-theta[j]))
    elif (abs(P_ex)>3 and abs(P_ex)<=5):
        mu=-0.5*P_ex+2.5
        Kxd1=abs(Kxd1 + 0.5*mu*(theta_objective[j]-theta[j]))
    elif (abs(P_ex)>4 and abs(P_ex)<=6.5):
        mu=(P_ex-4)/2.5
        Kxp1=abs(Kxp1 + 0.5*mu*(theta_objective[j]-theta[j]))
    elif (abs(P_ex)>6.5 and abs(P_ex)<=9):
        mu=(9-P_ex)/2.5
        Kxp1=abs(Kxp1 + 0.5*mu*(theta_objective[j]-theta[j])) 

    #z_vel[i]= z_vel[i-1] + z_acel[i]/2*dt
    #z[i]= z[i-1] + z_vel[i]/2*dt

    x_accel[j]=(math.sin(psi[j])*math.sin(phi[j])+math.cos(psi[j])*math.sin(theta[j])*math.cos(phi[j]))*Thrust[j]/m
  

    x_vel[j]=x_vel[j-1]+ x_accel[j]/2*dt
    
    #x_pos[j]=x_pos[j-1]+ x_accel[j]*dt*dt #x_vel[j]/2*dt
    x_pos[j]=x_pos[j-1]+ x_vel[j]*dt/2

    plt.subplot(3, 1, 1)
    plt.plot(j,x_accel[j],'or') #plot of the drone height vs time
    plt.subplot(3,1,2)
    plt.plot(j,x_vel[j],'ob')
    plt.subplot(3,1,3)
    plt.plot(j,x_pos[j],'og')
    plt.pause(0.1)
plt.show()





"""
##adaptación de fuzzy

if j>4 then
          
    P_ex= (theta_objetivo[j]-angulotheta_dcha[j])/(theta_objetivo[j])*100
    if (abs(P_ex)> 1 & abs(P_ex)<=3):
        mu=0.5*P_ex-0.5
        Kxd1=abs(Kxd1 + 0.5*mu*(theta_objetivo[j]-angulotheta_dcha[j]))
    elif (abs(P_ex)>3 & abs(P_ex)<=5):
        mu=-0.5*P_ex+2.5
        Kxd1=abs(Kxd1 + 0.5*mu*(theta_objetivo[j]-angulotheta_dcha[j]))
    elif (abs(P_ex)>4 & abs(P_ex)<=6.5):
        mu=(P_ex-4)/2.5
        Kxp1=abs(Kxp1 + 0.5*mu*(theta_objetivo[j]-angulotheta_dcha[j]))
    elif (abs(P_ex)>6.5 & abs(P_ex)<=9):
        mu=(9-P_ex)/2.5
        Kxp1=abs(Kxp1 + 0.5*mu*(theta_objetivo[j]-angulotheta_dcha[j]))    
      



if j>4:
    P_ey=(phi_objetivo[j]-phi_dcha[j])/(phi_objetivo[j])*100
    if (abs(P_ey)> 1 & abs(P_ey)<=3):
        mu=0.5*P_ey-0.5
        Kyd1=abs(Kyd1 + 0.5*mu*(phi_objetivo[j]-phi_dcha[j]))
    elif (abs(P_ey)>3 & abs(P_ey)<=5):
        mu=-0.5*P_ey+2.5
        Kyd1=abs(Kyd1 + 0.5*mu*(phi_objetivo[j]-phi_dcha[j]))
    elif (abs(P_ey)>4 & abs(P_ey)<=6.5):
        mu=(P_ey-4)/2.5
        Kyp1=abs(Kyp1 + 0.5*mu*(phi_objetivo[j]-phi_dcha[j]))
    elif (abs(P_ey)>6.5 & abs(P_ey)<=9)
        mu=(9-P_ey)/2.5
        yp1=abs(Kyp1 + 0.5*mu*(phi_objetivo[j]-phi_dcha[j]))    
      """




"""
for i in range(N):
    #DRONE height position, velocity and acceleration

    phi[i], theta[i], psi[i], phi_vel[i], theta_vel[i], psi_vel[i], Thrust, z_accel[i],error= quadrotor(phi[i-1], theta[i-1],psi[i-1],phi_vel[i-1], theta_vel[i-1], psi_vel[i-1], Height_objective, Thrust,z_vel[i-1],z[i-1],z_vel[i-2])
    
    z_vel[i]= z_vel[i-1] + z_accel[i]/2*dt
    
    z[i]= z[i-1] + z_vel[i]/2*dt
    
    plt.plot(i,z[i],'or') #plot of the drone height vs time
    plt.pause(0.1)
"""