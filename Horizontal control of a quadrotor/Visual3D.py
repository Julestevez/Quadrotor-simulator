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
    
    

    """ax.cla()
    ax.set_xlim3d(-100, 200)
    ax.set_ylim3d(-100,100)
    ax.set_zlim3d(0,100)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    for i in range(4):
        ax.plot([VecStart_x[i], VecEnd_x[i]], [VecStart_y[i],VecEnd_y[i]],zs=[VecStart_z[i],VecEnd_z[i]])
    
    ax.scatter3D(x_leader,y_leader,50,s=30)
    ax.scatter3D(x_follower,y_follower,50,s=30)
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
plt.show()"""