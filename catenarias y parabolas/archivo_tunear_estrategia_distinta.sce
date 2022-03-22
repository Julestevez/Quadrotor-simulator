//aquí pruebo si el cuadrotor apunta bien al ángulo que le indico con los valores P-I-D que he calculado en otro archivo
//AQUÍ ESTÁ EL FILTRO
//en el archivo 4-probarPIDtrayectoria tengo un ángulo deseado fijo.
//en el archivo 5-probarPIDtrayectoria tengo un ángulo progresivo
//en el 6, voy a cambiar. creo un while que para el programa cuando llega a la posición final.
//en el archivo 6 ya he conseguido que un dron vaya de x=240 a x=300 de una manera razonablemente buena. Ahora voy a hacer que uno le siga, y ver cómo se estira la catenaria entre medias.

//http://gafferongames.com/game-physics/integration-basics/

//lo que hago aquí es no imponer theta_objetivo, sino que se calcule solo (que vaya frenando y acelerando)
//http://linove.blogspot.com.es/2009/05/filter-design-using-scilab.html
//https://github.com/gibiansky/experiments/blob/master/quadcopter/matlab/simulate.m
clc; clear();close();
exec("catenaria3.sce");
exec("quadrotor.sce");
exec("descenso.sce");
exec("TensVerticales.sci");
exec("GeoPatMakeBlock.sci");
exec("GeoVerMakeBlock.sci");
exec("Euler2R.sci");
exec('anim_block2.sce');
exec("quadrotor_izq.sce");
exec("quadrotor_izqmov.sce");
exec("quadrotor_dcha2.sce");



//ahora, tengo que animar la catenaria
dt=0.1; dif_tiempo=0.1;
angulotheta_objetivo_dch=zeros(1,100);
angulotheta_objetivo_cen=zeros(1,100);
angulotheta_objetivo_izq=zeros(1,100);


//***PARAMÉTROS DINÁMICOS DE DRONES****
L=25; b = 1e-5; I = diag([0.5, 0.5, 1]); //L esta en centimetros
k=3e-5; m=0.5; //el momento de inercia está en [N·cm·s2]
Sab=200;
//PARROT
//L=17; b = 3.13e-5; I = diag([86, 86, 172]); //L esta en centimetros
//k=7.5e-5; m=0.38; 
//b=[N·cm·s2]  //k=[N·s2]



//***************************//
//DRON IZQUIERDA: defino parámetros
N=300; //N= número de elementos
phi_izq=zeros(1,N); angulotheta_izq=zeros(1,N); psi_izq=zeros(1,N); 
phi_vel_izq=zeros(1,N); angulotheta_vel_izq=zeros(1,N); psi_vel_izq=zeros(1,N);  
z_vel_izq=zeros(1,N); z_pos_izq=zeros(1,N); 
z_acel_izq=zeros(1,N); phi_vel_izq=zeros(1,N); angulotheta_vel_izq=zeros(1,N); psi_vel_izq=zeros(1,N); 
x_acel_izq=zeros(1,N); x_vel_izq=zeros(1,N); x_pos_izq=zeros(1,N); x_vel_izq=zeros(1,N);
y_acel_izq=zeros(1,N); y_vel_izq=zeros(1,N); y_pos_izq=zeros(1,N);


//***************************//
//DRONE DERECHA: defino parametros

phi_dcha=zeros(1,N); angulotheta_dcha=zeros(1,N); psi_dcha=zeros(1,N);
phi_vel_dcha=zeros(1,N);angulotheta_vel_dcha=zeros(1,N); psi_vel_dcha=zeros(1,N);
x_pos_dcha=zeros(1,N); y_pos_dcha=zeros(1,N); x_vel_dcha=zeros(1,N); y_vel_dcha=zeros(1,N);
x_acel_dcha=zeros(1,N); y_acel_dcha=zeros(1,N);
x_pos_dcha(1)=240; x_pos_dcha(2)=240; x_pos_dcha(3)=240;
x_deseado=270; x_vel_deseado=100;
Thrust_dcha=zeros(1,N);
Thrust_dcha(1)=11;
Thrust_dcha(2)=11;
Thrust_dcha(3)=11;
z_dcha=zeros(1,N); z_vel_dcha=zeros(1,N); z_acel_dcha=zeros(1,N);


//***************************//
//DRONE CENTRAL: defino parametros
//CONDICIONES INICIALES DE INESTABILIDAD
//phi(1)= 0.5; angulotheta(1)= -0.4; psi(1)= 0.6;
//phi(2)= 0.5; angulotheta(2)= -0.4; psi(2)= 0.6;
//phi(3)= 0.5; angulotheta(3)= -0.4; psi(3)= 0.6;

//ángulos iniciales distintos de 0
//phi(1)= -0.4; angulotheta(1)= 0.3; 
//phi(2)= -0.4; angulotehta(2)= 0.3;

//phi mueve en eje Y
//angulotheta mueve en eje X
phi=zeros(1,N);  angulotheta=zeros(1,N); psi=zeros(1,N);
phi_vel= zeros(1,N); angulotheta_vel= zeros(1,N); psi_vel= zeros(1,N); 
phi_acel= 0; angulotheta_acel= 0; psi_acel= 0;

x_vel2=zeros(1,N); x_pos2=zeros(1,N); x_acel2=zeros(1,N);
x_pos2(1)=120; x_pos2(2)=120; x_pos2(3)=120;
y_acel2=zeros(1,N); y_vel2=zeros(1,N); y_pos2=zeros(1,N);
z_pos2=zeros(1,N); z_vel2=zeros(1,N); z_acel2=zeros(1,N);
Thrust=zeros(1,N);
y_acel=zeros(1,N); y_vel=zeros(1,N); y_pos=zeros(1,N);
//**********************************//
x_pos_izq=zeros(1,N);
y_pos_izq=zeros(1,N);




 //valores del controlador PID
//Kp=4.05; Kd=20.64; Ki=0; //muy bueno
//Kp=4.6521; Kd=10.1373; Ki=0; //muy bueno
//Kp=3.6796; Kd=12.008; Ki=0; //muy bueno
//Kp=64.06; Kd=58.48; Ki=0;
Kp=140.6659; Kd=41.36; Ki=0;
w=0.005;

//Kp=2.008; Kd=4.9687; Ki=0;

xB1=120; xB2=120;
vectorX1=linspace(0,xB1,150);
vectorX2=linspace(0,xB2,150);
yB1=0; yB2=0;
z1=zeros(1,length(vectorX1));
z2=zeros(1,length(vectorX2));

matriz_y1=zeros(20,150);
matriz_y2=zeros(20,150);

        
[y1,x01,y01,c1] = catenaria3(xB1, yB1, Sab, vectorX1);
       
[y2,x02,y02,c2] = catenaria3(xB2, -yB1, Sab, vectorX2);
//vectorX2=vectorX1+120;
y2=y2+y1(150);
[yB2]= descenso(Sab, xB2,w);
yB2=-64.54; //OBTENIDO CON WOLFRAMALPHA

yB1=yB2;

//yB1=-7.5;   EL CONTROLADOR PID SIRVE PARA TODAS LAS ALTURAS, Y TAMBIÉN PARA TODAS LAS MASAS
//yB2=-7.5;


NudoBaja=yB1;


    vectorX2=linspace(0, xB2, 150);
    [y1,x01,y01,c1] = catenaria3(xB1, NudoBaja, Sab, vectorX1);
           
    [y2,x02,y02,c2] = catenaria3(xB2, -NudoBaja, Sab, vectorX2);
    vectorX2=vectorX1+120;
    y2=y2+y1(150);
    
     //tensiones verticales de los extremos [kg]
    [TensV_A, TensV_B1, TensH_A, TensH_B1]= TensVerticales(c1,y01,yB1);
    [TensV_B2, TensV_C, TensH_B2, TensH_C]= TensVerticales(c2,y02,yB2);
    
    Thrust(1)=TensV_B1+TensV_B2;
    Thrust(2)=TensV_B1+TensV_B2;
    Thrust(3)=TensV_B1+TensV_B2;
    
    Thrust_izq(1)=TensV_A;
    Thrust_izq(2)=TensV_A;
    Thrust_izq(3)=TensV_A;
   


//*********************************
//********************************
//*******************************
//PSO ALGORITHM
//PSO ALGORITHM
//**********************************************
N=4; //number of particles in the swarm
weight=0.9; //weight of the PSO algorithm //antes ponía 0.9. Si este valor sube mucho, Kd se vuelve grande y el sistema diverge
const_c1=0.01; //constant of the velocity algorithm
const_c2=0.01; //constant of the velocity algorithm
Number_Iter=85; //number of iterations
dim=3; //número de dimensión. Significa el número de variables Kp, Ki, Kd

//Initialize the parameter
fitness=zeros(N,Number_Iter);
R1=rand(dim,N); //Random numbers [0 1]
R2=rand(dim,N); //Random numbers [0 1]
current_fitness=zeros(N,1); //la current fitness va a ser el overshot

//initializing swarm and velocities and position
current_position=(rand(dim,N));// voy a multiplicar por 1 en lugar de por 10
velocity=3*rand(dim,N);
local_best_position = current_position;












x_deseado=270;
y_deseado=30;
y_vel_deseado=2;

//theta_objetivo=0.2;   
   j=2;
   for h=1:N
    //tengo que calcular el overshot con Kp, Ki, Kd distintos
    Kp1=abs(current_position(1,h));
    //Ki=abs(current_position(2,h));//AQUIII
    Kd1=abs(current_position(3,h));
       
       for (j=3:120)
       
       x_vel_deseado=10;
       x_acel_deseado=2;
       if (abs(x_pos_dcha(j-1))>x_deseado*3/5) then
           x_vel_deseado=0;
           x_acel_deseado=0;
       end
       theta_objetivo(j)=0.2;
//          Kd1=1; Kp1=0.8;
            //Kd1=1; Kp1=0.8; //valores muy buenos
            Ux_dcha(j)=x_acel_deseado + Kd1*(x_vel_deseado - x_vel_dcha(j-1)) + Kp1*(x_deseado-x_pos_dcha(j-1));
//            x_vel_deseado=0;
//            x_deseado=400;
            //Ux_dcha(j)=x_acel_dcha(j) + Kd1*(x_vel_deseado - x_vel_dcha(j-1)) + Kp1*(x_deseado-x_pos_dcha(j-1));
            //Uy=y_deseado + Kd1*(y_vel_deseado - y_vel_actual) + Kp1*(y_deseado-y_actual);
         a(j)=Ux_dcha(j)*m/(Thrust_dcha(j-1)*cos(angulotheta_dcha(j-1)));
         //chapuza
        if a(j)>0.5 then
            a(j)=0.5;
        elseif a(j)<-0.5 then
            a(j)=-0.5;
        end
        theta_objetivo(j)=asin(a(j));
        
        
        //DIRECCIÓN Y
       y_vel_deseado=2;
       y_acel_deseado=0.3;
        if (abs(y_pos_dcha(j-1))>y_deseado*3/5) then
           y_vel_deseado=0;
           y_acel_deseado=0;
       end
       
        Uy_dcha(j)=y_acel_deseado + Kd1*(y_vel_deseado - y_vel_dcha(j-1)) + Kp1*(y_deseado-y_pos_dcha(j-1));
        
        a_(j)=Uy_dcha(j)*m/(Thrust_dcha(j-1));
        
        if a_(j)>0.5 then
            a_(j)=0.5;
        elseif a_(j)<-0.5 then
            a_(j)=-0.5;
        end
        phi_objetivo(j)=-asin(a_(j));
        
       
    
    //funciona muy bien Kp 1 y Kd 0.8 para ir de 240 a 300
    //MUEVO EL DRON DCHA
    //*********************************
    //calculo tension variable TensV_A
    xB1=sqrt((x_pos_dcha(j-1)-x_pos2(j-2))^2 + (y_pos_dcha(j-1)-y_pos(j-2))^2);
    yB1=-64.5;
    [y1,x01,y01,c1] = catenaria3(xB1, yB1, Sab, vectorX1);
    [TensV_A, TensV_B1, TensH_A, TensH_B1]= TensVerticales(c1,y01,yB1);
    //plot(sqrt((x_pos_dcha(j)-x_pos2(j-1))^2 - (y_pos_dcha(j)-y_pos(j-1))^2),'or'); //sirve para ver la distancia real de los hilos
   
    
    
    [z_acel_dcha(j), phi_dcha(j), angulotheta_dcha(j), psi_dcha(j), phi_vel_dcha(j), angulotheta_vel_dcha(j), psi_vel_dcha(j), Thrust_dcha(j)] = quadrotor_dcha2(phi_dcha(j-1), angulotheta_dcha(j-1), psi_dcha(j-1), phi_vel_dcha(j-1),phi_vel_dcha(j-2), angulotheta_vel_dcha(j-1),angulotheta_vel_dcha(j-2), psi_vel_dcha(j-1),psi_vel_dcha(j-2), 0, TensV_A, z_vel_dcha(j-1),z_vel_dcha(j-2),z_dcha(j-1), TensH_B1, 0,theta_objetivo(j),phi_objetivo(j-1));

    x_acel_dcha(j)=(cos(psi_dcha(j))*cos(angulotheta_dcha(j)))*(TensH_B1)/m + (sin(psi_dcha(j))*sin(phi_dcha(j))+cos(psi_dcha(j))*sin(angulotheta_dcha(j))*cos(phi_dcha(j)))*Thrust_dcha(j)/m;
 
    x_vel_dcha(j)=x_vel_dcha(j-1)+ x_acel_dcha(j)/2*dt;
    //x_vel_dcha(j)=x_vel_dcha(j-1)+(x_acel_dcha(j)-x_acel_dcha(j-1))*dt;
    x_pos_dcha(j)=x_pos_dcha(j-1)+ x_vel_dcha(j)/2*dt;
    
    
    //muevo en Y
    y_acel_dcha(j)=(sin(psi_dcha(j))*sin(angulotheta_dcha(j))*cos(phi_dcha(j))-cos(psi_dcha(j))*sin(phi_dcha(j)))*Thrust_dcha(j)/m;
    
    y_vel_dcha(j)=y_vel_dcha(j-1)+ y_acel_dcha(j)/2*dt;
    
    y_pos_dcha(j)=y_pos_dcha(j-1)+ y_vel_dcha(j)/2*dt;
    


   
    
    //*********************************
    //LE SIGUE EL DRON CENTRAL
    //x_deseado2=max(x_pos_dcha)-120;
    x_deseado2=x_pos_dcha(j-1)-120; //si pongo x_deseado=x_pos_dcha(j-1)-120 -> el angulo tetha me sale muy irregular
    //si pongo x_deseado = 180, el ángulo me sale más fino
    y_deseado2=y_deseado;
    x_vel_deseado=10;
    x_acel_deseado=2;
    y_acel_deseado=0.3;
    y_vel_deseado=2;   
       
//theta_objetivo=0.2;   
    //CALCULO TENSIONES VARIABLES POR ESTAR MOVIÉNDOSE LOS DRONES
   xB2=sqrt((x_pos2(j-1)-x_pos_izq(j-2))^2 + (y_pos(j-1)-y_pos_izq(j-2))^2);
   yB2=-yB1;
   vectorX2=linspace(0, xB2, 150);
    [y2,x02,y02,c2] = catenaria3(xB2, -yB1, Sab, vectorX2);
    
    [TensV_B2, TensV_C, TensH_B2, TensH_C]= TensVerticales(c2,y02,yB2);

       
       if x_pos2(j)>x_deseado2*3/5 then
           x_vel_deseado=0;
           x_acel_deseado=0;
       end
       theta_objetivo2(j)=0.2;
//          Kd1=1; Kp1=0.8;
            //Kd1=1; Kp1=0.8; //valores muy buenos
            Ux(j)=x_acel_deseado + Kd1*(x_vel_deseado - x_vel2(j-1)) + Kp1*(x_deseado2-x_pos2(j-1));
//            x_vel_deseado=0;
//            x_deseado=400;
            //Ux_dcha(j)=x_acel_dcha(j) + Kd1*(x_vel_deseado - x_vel_dcha(j-1)) + Kp1*(x_deseado-x_pos_dcha(j-1));
            //Uy=y_deseado + Kd1*(y_vel_deseado - y_vel_actual) + Kp1*(y_deseado-y_actual);
         a2(j)=Ux(j)*m/(Thrust(j-1)*cos(angulotheta(j-1)));
         //chapuza
        if a2(j)>0.5 then
            a2(j)=0.5;
        elseif a2(j)<-0.5 then
            a2(j)=-0.5;
        end
        theta_objetivo2(j)=asin(a2(j));
        
        
        //DIRECCIÓN Y
        y_vel_deseado=2;
        if (abs(y_pos(j-1))>y_deseado2*3/5) then
           y_vel_deseado=0;
       end
       
        Uy(j)=y_acel_deseado + Kd1*(y_vel_deseado - y_vel(j-1)) + Kp1*(y_deseado2-y_pos(j-1));
        
        a_2(j)=Uy(j)*m/(Thrust(j-1));
        
        if a_2(j)>0.5 then
            a_2(j)=0.5;
        elseif a_2(j)<-0.5 then
            a_2(j)=-0.5;
        end
        phi_objetivo2(j)=-asin(a_2(j));
        
    
    [z_acel2(j), phi(j), angulotheta(j), psi(j), phi_vel(j), angulotheta_vel(j), psi_vel(j), Thrust(j)] = quadrotor_dcha2(phi(j-1), angulotheta(j-1), psi(j-1), phi_vel(j-1),phi_vel(j-2), angulotheta_vel(j-1),angulotheta_vel(j-2), psi_vel(j-1),psi_vel(j-2), 0, TensV_B2+TensV_B1, z_vel2(j-1),z_vel2(j-2),z_pos2(j-1), TensH_B1, TensH_B2 ,theta_objetivo2(j),phi_objetivo2(j));

    x_acel2(j)=(cos(psi(j))*cos(angulotheta(j)))*(TensH_B1-TensH_B2)/m + (sin(psi(j))*sin(phi(j))+cos(psi(j))*sin(angulotheta(j))*cos(phi(j)))*Thrust(j)/m;
 
    x_vel2(j)=x_vel2(j-1)+ x_acel2(j)/2*dt;
    //x_vel_dcha(j)=x_vel_dcha(j-1)+(x_acel_dcha(j)-x_acel_dcha(j-1))*dt;
    x_pos2(j)=x_pos2(j-1)+ x_vel2(j)/2*dt;
    
    
    distancia1(j)=x_pos_dcha(j)-x_pos2(j); //distancia entre el dron derecho y el central. Me sirve para calcular la tensión de la catenaria.
    
    //DIRECCIÓN Y
     y_acel(j)=(sin(psi(j))*sin(angulotheta(j))*cos(phi(j))-cos(psi(j))*sin(phi(j)))*Thrust(j)/m;
    y_vel(j)=y_vel(j-1)+ y_acel(j)/2*dt;
    y_pos(j)=y_pos(j-1)+ y_vel(j)/2*dt;
    
    
    

    

    
        
    //*********recalculo las tensiones verticales*****
//    [y2,x02,y02,c2] = catenaria3(distancia2(j), -NudoBaja, Sab, vectorX2);
//    //tensiones verticales de los extremos [kg]
//    [TensV_B2, TensV_C, TensH_B2, TensH_C]= TensVerticales(c2,y02,yB2);
//    TensV_C=TensV_A; //CHAPUZAAAA
    end //WHILE1 //CIERO LA J, QUE RECORRE LAS VARIABLES DE LOS DRONES
    distancia_deseada=sqrt(x_deseado^2 + y_deseado^2);
    distancia_real=sqrt((max(x_pos_dcha))^2 + (max(y_pos_dcha))^2);
     current_fitness(h)=abs(max(distancia_deseada-distancia_real));
     disp(current_fitness);
end //AQUÍ CIERRO LA VARIABLE QUE RECORRE EL NÚMERO DE PARTÍCULAS



//***************************************
//******************JUEVES

local_best_fitness = current_fitness;
global_best_fitness=min(local_best_fitness);
//
for i=1:N
    //aquí elijo cuál es la posición con el mejor valor de overshot
    g=find(current_fitness==min(current_fitness));
    global_best_position(:,i)=local_best_position(:,g);
    //estoy haciendo que la global_best_position tenga todos los Kp, Ki, Kd que genera el menor overshot
end
    
//velocity update
velocity = weight*velocity + const_c1*(R1.*(local_best_position-current_position)) + const_c2*(R2.*(global_best_position - current_position));

//swarm update
current_position = current_position + velocity;

//evaluate a new swarm
iter=0; //iterations counter


disp('segunda parte');
//tercera parte del PSO: hago todas las iteraciones
while(iter < = 100)
    iter = iter + 1;
    
    for i = 1:N
        Kp=abs(current_position(1,i)); 
        //Ki=abs(current_position(2,i));
        Kd=abs(current_position(3,i));
       // disp(Kp);disp(Kd);
    
         //evalúo ne las condiciones actuales. Calculo la respuesta para Kp-Kd
    for j=3:120
     x_vel_deseado=10;
       x_acel_deseado=2;
       if (abs(x_pos_dcha(j-1))>x_deseado*3/5) then
           x_vel_deseado=0;
           x_acel_deseado=0;
       end
       theta_objetivo(j)=0.2;
//          Kd1=1; Kp1=0.8;
            //Kd1=1; Kp1=0.8; //valores muy buenos
            Ux_dcha(j)=x_acel_deseado + Kd1*(x_vel_deseado - x_vel_dcha(j-1)) + Kp1*(x_deseado-x_pos_dcha(j-1));
//            x_vel_deseado=0;
//            x_deseado=400;
            //Ux_dcha(j)=x_acel_dcha(j) + Kd1*(x_vel_deseado - x_vel_dcha(j-1)) + Kp1*(x_deseado-x_pos_dcha(j-1));
            //Uy=y_deseado + Kd1*(y_vel_deseado - y_vel_actual) + Kp1*(y_deseado-y_actual);
         a(j)=Ux_dcha(j)*m/(Thrust_dcha(j-1)*cos(angulotheta_dcha(j-1)));
         //chapuza
        if a(j)>0.5 then
            a(j)=0.5;
        elseif a(j)<-0.5 then
            a(j)=-0.5;
        end
        theta_objetivo(j)=asin(a(j));
        
        
        //DIRECCIÓN Y
       y_vel_deseado=2;
       y_acel_deseado=0.3;
        if (abs(y_pos_dcha(j-1))>y_deseado*3/5) then
           y_vel_deseado=0;
           y_acel_deseado=0;
       end
       
        Uy_dcha(j)=y_acel_deseado + Kd1*(y_vel_deseado - y_vel_dcha(j-1)) + Kp1*(y_deseado-y_pos_dcha(j-1));
        
        a_(j)=Uy_dcha(j)*m/(Thrust_dcha(j-1));
        
        if a_(j)>0.5 then
            a_(j)=0.5;
        elseif a_(j)<-0.5 then
            a_(j)=-0.5;
        end
        phi_objetivo(j)=-asin(a_(j));
        
       
    
    //funciona muy bien Kp 1 y Kd 0.8 para ir de 240 a 300
    //MUEVO EL DRON DCHA
    //*********************************
    //calculo tension variable TensV_A
    xB1=sqrt((x_pos_dcha(j-1)-x_pos2(j-2))^2 + (y_pos_dcha(j-1)-y_pos(j-2))^2);
    yB1=-64.5;
    [y1,x01,y01,c1] = catenaria3(xB1, yB1, Sab, vectorX1);
    [TensV_A, TensV_B1, TensH_A, TensH_B1]= TensVerticales(c1,y01,yB1);
    //plot(sqrt((x_pos_dcha(j)-x_pos2(j-1))^2 - (y_pos_dcha(j)-y_pos(j-1))^2),'or'); //sirve para ver la distancia real de los hilos
   
    
    
    [z_acel_dcha(j), phi_dcha(j), angulotheta_dcha(j), psi_dcha(j), phi_vel_dcha(j), angulotheta_vel_dcha(j), psi_vel_dcha(j), Thrust_dcha(j)] = quadrotor_dcha2(phi_dcha(j-1), angulotheta_dcha(j-1), psi_dcha(j-1), phi_vel_dcha(j-1),phi_vel_dcha(j-2), angulotheta_vel_dcha(j-1),angulotheta_vel_dcha(j-2), psi_vel_dcha(j-1),psi_vel_dcha(j-2), 0, TensV_A, z_vel_dcha(j-1),z_vel_dcha(j-2),z_dcha(j-1), TensH_B1, 0,theta_objetivo(j),phi_objetivo(j-1));

    x_acel_dcha(j)=(cos(psi_dcha(j))*cos(angulotheta_dcha(j)))*(TensH_B1)/m + (sin(psi_dcha(j))*sin(phi_dcha(j))+cos(psi_dcha(j))*sin(angulotheta_dcha(j))*cos(phi_dcha(j)))*Thrust_dcha(j)/m;
 
    x_vel_dcha(j)=x_vel_dcha(j-1)+ x_acel_dcha(j)/2*dt;
    //x_vel_dcha(j)=x_vel_dcha(j-1)+(x_acel_dcha(j)-x_acel_dcha(j-1))*dt;
    x_pos_dcha(j)=x_pos_dcha(j-1)+ x_vel_dcha(j)/2*dt;
    
    
    //muevo en Y
    y_acel_dcha(j)=(sin(psi_dcha(j))*sin(angulotheta_dcha(j))*cos(phi_dcha(j))-cos(psi_dcha(j))*sin(phi_dcha(j)))*Thrust_dcha(j)/m;
    
    y_vel_dcha(j)=y_vel_dcha(j-1)+ y_acel_dcha(j)/2*dt;
    
    y_pos_dcha(j)=y_pos_dcha(j-1)+ y_vel_dcha(j)/2*dt;
    


   
    
    //*********************************
    //LE SIGUE EL DRON CENTRAL
    //x_deseado2=max(x_pos_dcha)-120;
    x_deseado2=x_pos_dcha(j-1)-120; //si pongo x_deseado=x_pos_dcha(j-1)-120 -> el angulo tetha me sale muy irregular
    //si pongo x_deseado = 180, el ángulo me sale más fino
    y_deseado2=y_deseado;
    x_vel_deseado=10;
    x_acel_deseado=2;
    y_acel_deseado=0.3;
    y_vel_deseado=2;   
       
//theta_objetivo=0.2;   
    //CALCULO TENSIONES VARIABLES POR ESTAR MOVIÉNDOSE LOS DRONES
   xB2=sqrt((x_pos2(j-1)-x_pos_izq(j-2))^2 + (y_pos(j-1)-y_pos_izq(j-2))^2);
   yB2=-yB1;
   vectorX2=linspace(0, xB2, 150);
    [y2,x02,y02,c2] = catenaria3(xB2, -yB1, Sab, vectorX2);
    
    [TensV_B2, TensV_C, TensH_B2, TensH_C]= TensVerticales(c2,y02,yB2);

       
       if x_pos2(j)>x_deseado2*3/5 then
           x_vel_deseado=0;
           x_acel_deseado=0;
       end
       theta_objetivo2(j)=0.2;
//          Kd1=1; Kp1=0.8;
            //Kd1=1; Kp1=0.8; //valores muy buenos
            Ux(j)=x_acel_deseado + Kd1*(x_vel_deseado - x_vel2(j-1)) + Kp1*(x_deseado2-x_pos2(j-1));
//            x_vel_deseado=0;
//            x_deseado=400;
            //Ux_dcha(j)=x_acel_dcha(j) + Kd1*(x_vel_deseado - x_vel_dcha(j-1)) + Kp1*(x_deseado-x_pos_dcha(j-1));
            //Uy=y_deseado + Kd1*(y_vel_deseado - y_vel_actual) + Kp1*(y_deseado-y_actual);
         a2(j)=Ux(j)*m/(Thrust(j-1)*cos(angulotheta(j-1)));
         //chapuza
        if a2(j)>0.5 then
            a2(j)=0.5;
        elseif a2(j)<-0.5 then
            a2(j)=-0.5;
        end
        theta_objetivo2(j)=asin(a2(j));
        
        
        //DIRECCIÓN Y
        y_vel_deseado=2;
        if (abs(y_pos(j-1))>y_deseado2*3/5) then
           y_vel_deseado=0;
       end
       
        Uy(j)=y_acel_deseado + Kd1*(y_vel_deseado - y_vel(j-1)) + Kp1*(y_deseado2-y_pos(j-1));
        
        a_2(j)=Uy(j)*m/(Thrust(j-1));
        
        if a_2(j)>0.5 then
            a_2(j)=0.5;
        elseif a_2(j)<-0.5 then
            a_2(j)=-0.5;
        end
        phi_objetivo2(j)=-asin(a_2(j));
        
    
    [z_acel2(j), phi(j), angulotheta(j), psi(j), phi_vel(j), angulotheta_vel(j), psi_vel(j), Thrust(j)] = quadrotor_dcha2(phi(j-1), angulotheta(j-1), psi(j-1), phi_vel(j-1),phi_vel(j-2), angulotheta_vel(j-1),angulotheta_vel(j-2), psi_vel(j-1),psi_vel(j-2), 0, TensV_B2+TensV_B1, z_vel2(j-1),z_vel2(j-2),z_pos2(j-1), TensH_B1, TensH_B2 ,theta_objetivo2(j),phi_objetivo2(j));

    x_acel2(j)=(cos(psi(j))*cos(angulotheta(j)))*(TensH_B1-TensH_B2)/m + (sin(psi(j))*sin(phi(j))+cos(psi(j))*sin(angulotheta(j))*cos(phi(j)))*Thrust(j)/m;
 
    x_vel2(j)=x_vel2(j-1)+ x_acel2(j)/2*dt;
    //x_vel_dcha(j)=x_vel_dcha(j-1)+(x_acel_dcha(j)-x_acel_dcha(j-1))*dt;
    x_pos2(j)=x_pos2(j-1)+ x_vel2(j)/2*dt;
    
    
    distancia1(j)=x_pos_dcha(j)-x_pos2(j); //distancia entre el dron derecho y el central. Me sirve para calcular la tensión de la catenaria.
    
    //DIRECCIÓN Y
     y_acel(j)=(sin(psi(j))*sin(angulotheta(j))*cos(phi(j))-cos(psi(j))*sin(phi(j)))*Thrust(j)/m;
    y_vel(j)=y_vel(j-1)+ y_acel(j)/2*dt;
    y_pos(j)=y_pos(j-1)+ y_vel(j)/2*dt;
    
    
    
    //*****recalculo las tensiones verticales******
//    [y1,x01,y01,c1] = catenaria3(distancia1(j), NudoBaja, Sab, vectorX1);
//    [TensV_A, TensV_B1, TensH_A, TensH_B1]= TensVerticales(c1,y01,yB1);
//    plot(j,TensV_A,'or');  
    
        disp("llega");
    
        
    //*********recalculo las tensiones verticales*****
//    [y2,x02,y02,c2] = catenaria3(distancia2(j), -NudoBaja, Sab, vectorX2);
//    //tensiones verticales de los extremos [kg]
//    [TensV_B2, TensV_C, TensH_B2, TensH_C]= TensVerticales(c2,y02,yB2);
//    TensV_C=TensV_A; //CHAPUZAAAA
    end

        //cálculo del overshot
    distancia_deseada=sqrt(x_deseado^2 + y_deseado^2);
    distancia_real=sqrt((max(x_pos_dcha))^2 + (max(y_pos_dcha))^2);
     current_fitness(h)=abs(max(distancia_deseada-distancia_real));
     disp(current_fitness);
    end
               


    for var = 1 : N
    if current_fitness(var) < local_best_fitness(var) then
      local_best_fitness(var) = current_fitness(var);
      local_best_position(var) = current_position(var);  
    end       
    end
   
   current_glob_bestfitness = min(local_best_fitness);
  
   if (current_glob_bestfitness < global_best_fitness)
          global_best_fitness = current_glob_bestfitness;
      
      for var2=1:N
             //aquí elijo cuál es la posición con el mejor valor de overshot
             g=find(current_fitness==min(current_fitness));
             global_best_position(:,var2)=local_best_position(:,g);
            //estoy haciendo que la global_best_position tenga todos los Kp, Ki, Kd que genera el menor overshot   
     end
    end
    
    velocity = weight * velocity + const_c1*(R1.*(local_best_position - current_position)) + const_c2*(R2.*(global_best_position - current_position));
   
    current_position = current_position + velocity;
    
     
end





//end  //WHILE2
//end //WHILE3
//end //WHILE4
//Kp=140.6659; Kd=41.36; Ki=0; buenos
//Kp=64.06; Kd=58.48; Ki=0;
// Kp=141.86; Kd=43.36; Ki=0;
//http://www.mathworks.com/matlabcentral/fileexchange/40052-pd-control-of-quadrotor/content/PDControlOfQuadrotor/quad_control_Main.m
//http://www.sersc.org/journals/IJCA/vol6_no5/32.pdf


//FILTRO
//
//x = [zeros(130,1);
//  1.2*ones(370,1) + 0.2*cos(2*%pi*[0:369]'/90)];
//y = x + 0.1*rand(500,1,'normal');
//[h,hm,fr]=wfir("lp",33,[.05 0],"hm",[0 0]);
//z = filter(h,1,y);

