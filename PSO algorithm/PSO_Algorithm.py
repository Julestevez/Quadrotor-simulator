import numpy as np

#PSO ALGORITHM
#Quiero comprobar si encuentra el punto más bajo de una U
#**********************************************
N=10 #number of particles in the swarm
weight=1 #weight of the PSO algorithm #antes ponÃ­a 0.9. Si este valor sube mucho, Kd se vuelve grande y el sistema diverge
const_c1=2 #constant of the velocity algorithm. El valor inicial es 2
const_c2=2 #constant of the velocity algorithm. El valor inicial es 2
Number_Iter=300 #number of iterations
dim=1 #nÃºmero de dimensiÃ³n. Significa el nÃºmero de variables Kp, Ki, Kd

#Initialize the parameter
fitness=np.zeros((N,Number_Iter))
R1=np.random.rand(dim,N) #Random numbers [0 1]
R2=np.random.rand(dim,N) #Random numbers [0 1]
current_fitness=np.zeros(N) #la current fitness va a ser el overshot

#initializing swarm and velocities and position
current_position=2*np.random.rand(dim,N)# voy a multiplicar por 1 en lugar de por 10
velocity=3*np.random.rand(dim,N)
local_best_position = current_position
global_best_position = np.zeros((dim,N)) #ESTO ES NUEVO


#evaluate initial population

#tengo que calcular el overshot con Kp, Ki, Kd distintos
x1=current_position
y=x1**2 #forma de la parabola
        

current_fitness=y;#lo que quiero optimizar es que la funciÃ³n me halle el punto mÃ¡s bajo
print(current_fitness)


local_best_fitness = current_fitness
global_best_fitness=np.amin(local_best_fitness)

for i in range(N):
    #aquÃ­ elijo cuÃ¡l es la posiciÃ³n con el mejor valor de overshot
    
    #g=np.where(current_fitness==np.argmin(current_fitness))
    g=np.argmin(current_fitness) #devuelve el índice del valor mínimo
    global_best_position[:,i]=local_best_position[:,g]
    #estoy haciendo que la global_best_position tenga todos los Kp, Ki, Kd que genera el menor overshot

    
#velocity update
velocity = weight*velocity + const_c1*(R1*(local_best_position-current_position)) + const_c2*(R2*(global_best_position - current_position));

#swarm update
current_position = current_position + velocity

#evaluate a new swarm
iter=0; #iterations counter
#
while(iter <= Number_Iter):
    iter = iter + 1
       
    x1=current_position
    y= x1**2
                  
    current_fitness=y
    print(iter); print(i)
            

for var in range (N):
    if current_fitness[var] < local_best_fitness[var]:
        local_best_fitness[var] = current_fitness[var]
        local_best_position[var] = current_position[var]
       

   
    current_glob_bestfitness = min(local_best_fitness)
  
    if (current_glob_bestfitness < global_best_fitness):
        global_best_fitness = current_glob_bestfitness
      
        for var2 in range(N):
             #aquÃ­ elijo cuÃ¡l es la posiciÃ³n con el mejor valor de overshot
             g=find(current_fitness==min(current_fitness))
             global_best_position[var2]=local_best_position[g]
            #estoy haciendo que la global_best_position tenga todos los Kp, Ki, Kd que genera el menor overshot   
     
    
    velocity = weight * velocity + const_c1*(R1(local_best_position - current_position)) + const_c2*(R2(global_best_position - current_position));
   
    current_position = current_position + velocity
    plot(x1,y,'or')
    y1=min(y)
