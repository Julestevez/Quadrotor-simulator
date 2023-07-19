#main file

#This code represents the control of a single quadrotor in a horizontal motion in X-Y directions

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpmath import *
from catenary import catenary



N=3000 #number of steps in simulation
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

for j in range(N):
    #I will create two catenaries (x1,y1)-(x2,y2)-(x3,y3). Some of the coordinates are variable in time
    
    #CABLE COORDINATES
    x1 = 110 + 30*math.sin(0.2*j)
    y1 = 0

    x2 = 50
    y2 = 20*math.sin(0.1*j)
    
    x3 = 0
    y3 = 0
    
    #Euclidean horizontal distance between endpoints [cm]
    HorDif1=math.sqrt(math.pow((x1-x2),2)+math.pow((y1-y2),2)) 
    HorDif2=math.sqrt(math.pow((x2-x3),2)+math.pow((y2-y3),2)) 

    VerDif=0 #vertical distance between endpoints [cm]
    S=120 #length of the catenary in cm
    w=0.005 #weight per unit length [kg/cm]
    
    
    vectorX1=np.linspace(0,x1-x2,100) #X coordinates of the first catenary
    vectorY1=np.linspace(0,y1-y2,100) #Y ""     ""      ""      ""
    vectorX2=np.linspace(0,x2-x3,100) #X coordinates of the second catenary
    vectorY2=np.linspace(0,y2-y3,100) #Y ""     ""      ""      ""
    
    #Create the equations of the catenaries
    z1,x01,z_1,c1 = catenary(HorDif1, VerDif, S, np.linspace(0,HorDif1,100))
    z2,x02,z_2,c2 = catenary(HorDif2, VerDif, S, np.linspace(0,HorDif2,100))
   
    
    #Visualize the catenaries
    ax.cla()
    ax.set_xlim3d(-50, 150)
    ax.set_ylim3d(-50, 50)
    ax.set_zlim3d(-50, 50)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Plot of catenaries')
    ax.plot(vectorX1+x2, vectorY1+y2, z1) # ax.plot requires that all the arguments have the same size
    ax.plot(vectorX2, vectorY2, z2)
    
    plt.pause(0.1)
plt.show()

