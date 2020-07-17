#3D Visualization

import matplotlib.pyplot as plt
import math



def Visual3D(theta_leader, phi_leader, x_leader, y_leader, theta_follower, phi_follower, x_follower, y_follower):
      
    #plot of the UAV
    #DRONE1
    X1=x_leader-20*math.cos(theta_leader)
    X2=x_leader
    X3=x_follower-20*math.cos(theta_follower)
    X4=x_follower

    Y1=y_leader
    Y2=y_leader-20*math.cos(phi_leader)
    Y3=y_follower
    Y4=y_follower-20*math.cos(phi_follower)

    Z1=50+20*math.sin(theta_leader)
    Z2=50+20*math.sin(phi_leader)
    Z3=50+20*math.sin(theta_follower)
    Z4=50+20*math.sin(phi_follower)

    

    #DRONE2
    X5=x_leader+20*math.cos(theta_leader)
    X6=x_leader
    X7=x_follower+20*math.cos(theta_follower)
    X8=x_follower

    Y5=y_leader
    Y6=y_leader+20*math.cos(phi_leader)
    Y7=y_follower
    Y8=y_follower+20*math.cos(phi_follower)

    Z5=50-20*math.sin(theta_leader)
    Z6=50-20*math.sin(phi_leader)
    Z7=50-20*math.sin(theta_follower)
    Z8=50-20*math.sin(phi_follower)
    
    VecStart_x=[X1, X2, X3, X4]
    VecStart_y=[Y1, Y2, Y3, Y4]
    VecStart_z=[Z1, Z2, Z3, Z4]
    VecEnd_x= [X5, X6, X7, X8]
    VecEnd_y= [Y5, Y6, Y7, Y8]
    VecEnd_z= [Z5, Z6, Z7, Z8]

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