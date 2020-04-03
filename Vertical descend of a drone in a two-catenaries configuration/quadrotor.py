import math
import numpy as np

def quadrotor(phi_ant, theta_ant, psi_ant, phi_vel_ant,theta_vel_ant,psi_vel_ant, Ymax, Thrust,z_vel_ant,z_ant,z_vel_2ant):
    inputs=[0]*4
    dt=0.1
    L=25 #L esta en centimetros
    b = 1e-5 
    I = np.diag([5e-7, 5e-7, 10e-7]) 
    k=3e-5
    m=0.5
    kd=0.25
    g=9.80

    #PID Values
    Kp, Ki, Kd = 4.05, 0, 20.64

    I = np.diag([5e-7, 5e-7, 10e-7]) #inertia moment of the drone
    Torque=[0]*4

    #pid controller. Roll = 0
    err_phi=-phi_ant 
    Torque[1]=(err_phi*Kp + err_phi*Ki*dt - phi_vel_ant*Kd*dt )* I[0,0] #+ TensionHoriz*math.cos(phi_ant)*L/I[0,0]

    #PID Controller. Pitch = 0
    err_theta=-theta_ant 
    Torque[2]=(err_theta*Kp + err_theta*Ki*dt - theta_vel_ant*Kd*dt) * I[1,1]
    
    #PID Controller. Yaw = 0
    err_yaw=-psi_ant 
    Torque[3]=(err_yaw*Kp + err_yaw*Ki*dt - psi_vel_ant*Kd*dt ) * I[2,2] 
    
    #PID Controller. Height error = 0
    err_height=Ymax-z_ant
    Torque[0]=err_height*Kp + err_height*Ki*dt - (z_vel_ant-z_vel_2ant)*Kd*dt + (m*g)/(math.cos(phi_ant)*math.cos(theta_ant)) 
    if Torque[0]>20:
        Torque[0]=20
    elif Torque[0]<3:
        Torque[0]=3
      
    #calculo los inputs
    inputs[0]= 1/(4*k)*Torque[0] - 1/(2*k*L)*Torque[2] - 1/(4*b)*Torque[3] 
    inputs[1]= 1/(4*k)*Torque[0] - 1/(2*k*L)*Torque[1] + 1/(4*b)*Torque[3] 
    inputs[2]= 1/(4*k)*Torque[0] + 1/(2*k*L)*Torque[2] - 1/(4*b)*Torque[3] 
    inputs[3]= 1/(4*k)*Torque[0] + 1/(2*k*L)*Torque[1] + 1/(4*b)*Torque[3] 
    
    phi_acel=    Torque[1]/I[0,0] 
    theta_acel=  Torque[2]/I[1,1] 
    psi_acel=    Torque[3]/I[2,2] 
    
    phi_vel=     phi_vel_ant + phi_acel*dt 
    theta_vel=   theta_vel_ant + theta_acel*dt 
    psi_vel=     psi_vel_ant + psi_acel*dt 
    
    phi=         phi_ant + phi_vel*dt 
    theta=       theta_ant + theta_vel*dt 
    psi=         psi_ant + psi_vel*dt 
    
    z_acel= -g + (math.cos(theta_ant)*math.cos(phi_ant))*Torque[0]/m 

    return phi, theta, psi, phi_vel, theta_vel, psi_vel, Torque[0], z_acel, err_height

