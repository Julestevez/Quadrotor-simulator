#3D Visualization

import matplotlib.pyplot as plt
import math



def Visual3D(theta, phi, x_pos, y_pos):
    L=7 #length of the arm (for VISUALIZATION)

    #plot of the UAV
    X1=x_pos-L*math.cos(theta)
    X2=x_pos
    X3=x_pos+L*math.cos(theta)
    X4=x_pos
    
    Y1=y_pos
    Y2=y_pos-L*math.cos(phi)
    Y3=y_pos
    Y4=y_pos+L*math.cos(phi)
    

    Z1=50+L*math.sin(theta)
    Z2=50+L*math.sin(phi)
    Z3=50-L*math.sin(theta)
    Z4=50-L*math.sin(phi)
    
      
    VecStart_x=[X1, X2]
    VecStart_y=[Y1, Y2]
    VecStart_z=[Z1, Z2]
    VecEnd_x= [X3, X4]
    VecEnd_y= [Y3, Y4]
    VecEnd_z= [Z3, Z4]

    return VecStart_x,VecStart_y,VecStart_z, VecEnd_x,VecEnd_y,VecEnd_z
    
