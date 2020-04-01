import numpy as np
import matplotlib.pyplot as plt

#PSO ALGORITHM
#Quiero comprobar si encuentra el punto más bajo de una U
#**********************************************
N=2 #number of particles in the swarm
weight=1 #weight of the PSO algorithm.
const_c1=2 #constant of the velocity algorithm
const_c2=2 #constant of the velocity algorithm
Number_Iter=300 #number of iterations
dim=1 #number of dimensions. It means the number of variables to find. If we look for a, b, c, dim=3

#Initialize the parameter
fitness=np.zeros((N,Number_Iter))
R1=np.random.rand(dim,N) #Random numbers [0 1]
R2=np.random.rand(dim,N) #Random numbers [0 1]
current_fitness=np.zeros(N) #fitness is going to be defined as the overshot

#initializing swarm and velocities and position
current_position=2*np.random.rand(dim,N)# initial positions of the particles
velocity=3*np.random.rand(dim,N)        # initial velocities of the particles
local_best_position = current_position  
#global_best_position = np.zeros((dim,N)) #ESTO ES NUEVO


#evaluate initial population
x1=current_position
y=x1**2 #parabole
        

current_fitness=y;#lo que quiero optimizar es que la funciÃ³n me halle el punto mÃ¡s bajo
print(current_fitness)


local_best_fitness = current_fitness
global_best_fitness=np.amin(local_best_fitness)

for i in range(N):
    g=np.argmin(current_fitness) #devuelve el índice del valor mínimo
    global_best_position[0,i]=local_best_position[0,g] 
        
#velocity update
velocity = weight*velocity + const_c1*(R1*(local_best_position-current_position)) + const_c2*(R2*(global_best_position - current_position));

#swarm update
current_position = current_position + velocity #new positions of the particles

#evaluate a new swarm
iter=0; #iterations counter

while(iter <= Number_Iter):
    iter = iter + 1
       
    x1=current_position #10 rand particles which represent x values
    y= x1**2            #the y values corresponding to x1 particles
                  
    current_fitness=y   #now, y values are current_fitness.
    print("iteracion:",iter) 
               
    for var in range (N):
        if current_fitness[0,var] < local_best_fitness[0,var]: #current fitness tiene unidades de (1,X). Así que hay que poner [0,var]
            local_best_fitness[0,var] = current_fitness[0,var]
            local_best_position[0,var] = current_position[0,var]
       
    current_glob_bestfitness = np.amin(local_best_fitness)
  
    if (current_glob_bestfitness < global_best_fitness):
        global_best_fitness = current_glob_bestfitness
      
        for var2 in range(N):
            g=np.argmin(current_fitness)
            global_best_position[0,var2]=local_best_position[0,g]
            
    velocity = weight * velocity + const_c1*(R1*(local_best_position - current_position)) + const_c2*(R2*(global_best_position - current_position))
   
    current_position = current_position + velocity
    plt.plot(x1,y,'.r')
    plt.pause(0.1)

    if iter>150:
        plt.plot(x1,y,'.b')
        plt.pause(0.1)
   
     
plt.show()
#y1=min(y)
