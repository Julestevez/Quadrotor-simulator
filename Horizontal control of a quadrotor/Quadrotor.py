import math
import numpy as np

#***Dynamic parameters of DRONE****
L=0.25 # [m]
b = 1e-5 
I = np.diag([5e-3, 5e-3, 10e-3])  # [kgm2]
k=3e-5
m=0.5 #[kg]
kd=0.25
g=9.80 #m/s2
dt=0.1 #s


def angle_objective(desired_states, pos, vel, angle, Thrust, Kp_fuzzy,Kd_fuzzy):
    print(desired_states)
    U=desired_states[2] + Kd_fuzzy*(int(desired_states[1]) - vel) + Kp_fuzzy*(int(desired_states[0])-pos)
    a=U*m/(Thrust*math.cos(angle))
    #chapuza
    if a>0.7:
        a=0.7
    elif a<-0.7:
        a=-0.7
        
    angle_objective=math.asin(a)
    return angle_objective




def quadrotor(States, desired_states, Thrust):
    inputs=[0]*4
    #desired_states=list(map(int, desired_states))
    #PID Values
    Kp, Ki, Kd = 4.05, 0, 20.64

    Torque=[0]*4

    #pid controller. Roll = 0
    err_phi=desired_states[1]-States[1] 
    Torque[1]=(err_phi*Kp + err_phi*Ki*dt - States[4]*Kd*dt )* I[0,0] #+ TensionHoriz*math.cos(phi_ant)*L/I[0,0]

    #PID Controller. Pitch = 0
    err_theta=desired_states[0]-States[0]
    Torque[2]=(err_theta*Kp + err_theta*Ki*dt - States[3]*Kd*dt) * I[1,1]
    
    #PID Controller. Yaw = 0
    err_yaw=desired_states[2]-States[2]
    Torque[3]=(err_yaw*Kp + err_yaw*Ki*dt - States[5]*Kd*dt ) * I[2,2] 
    
    #PID Controller. Height error = 0
    err_height=desired_states[3]-States[8]
    Torque[0]=err_height*Kp + err_height*Ki*dt - States[8]*Kd*dt + (m*g)/(math.cos(States[1])*math.cos(States[0])) 
    if Torque[0]>8:
        Torque[0]=8
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

    theta_vel, theta = EulerIntegration(theta_acel,States[3],States[0])
    phi_vel, phi =     EulerIntegration(phi_acel,States[4],States[1])
    psi_vel, psi =     EulerIntegration(psi_acel,States[5],States[2])
    
    z_accel= -g + (math.cos(theta)*math.cos(phi))*Torque[0]/m
    x_accel=(math.sin(psi)*math.sin(phi)+math.cos(psi)*math.sin(theta)*math.cos(phi))*Torque[0]/m

    results= [theta, phi, psi, theta_vel, phi_vel, psi_vel]

    return results, Torque[0], z_accel, x_accel

  
    
def EulerIntegration(accel, vel, pos):
    vel_new = vel + accel/2*dt
     
    pos_new = pos+ vel_new/2*dt

    return vel_new, pos_new

