clc; clear();close();
exec("catenaria3.sce");
exec("quadrotor.sce");
exec("descenso.sce");
exec("TensVerticales.sci");
exec("quadrotor_izq.sce");
exec("quadrotor_izqmov.sce");
exec("quadrotor_dcha2.sce");
exec("seguidores.sce");


Circuito_x=linspace(0,500,600);
Circuito_y=zeros(1,400);
Circuito_y(1:150)=linspace(0,50,150);
Circuito_y(151:300)=ones(1,150)*50;
Circuito_y(300:500)=linspace(50,-100,201);
Circuito_y(500:600)=linspace(-100,50,101);
//Circuito_y(500:600)=linspace(-100,-50,101);
//Circuito_y(500:600)=ones(1,101)*(-100);
dt=0.1; dif_tiempo=0.1;
angulotheta_objetivo_dch=zeros(1,100);
angulotheta_objetivo_cen=zeros(1,100);
angulotheta_objetivo_izq=zeros(1,100);


//***PARAMÉTROS DINÁMICOS DE DRONES****
L=25; b = 1e-5; I = diag([0.5, 0.5, 1]); //L esta en centimetros
k=3e-5; m=0.5; //el momento de inercia está en [N·cm·s2]
Sab=240;



N=600; //N= número de elementos
angulo_max=0.7; //estaba 0.7
//***************************//
//DRONE DERECHA: defino parametros

phi_dcha=zeros(1,N); angulotheta_dcha=zeros(1,N); psi_dcha=zeros(1,N);
phi_vel_dcha=zeros(1,N);angulotheta_vel_dcha=zeros(1,N); psi_vel_dcha=zeros(1,N);
x_pos_dcha=zeros(1,N); y_pos_dcha=zeros(1,N); x_vel_dcha=zeros(1,N); y_vel_dcha=zeros(1,N);
x_acel_dcha=zeros(1,N); y_acel_dcha=zeros(1,N);
x_pos_dcha(1)=0; x_pos_dcha(2)=0; x_pos_dcha(3)=0;
y_pos_dcha(1)=0; y_pos_dcha(2)=0; y_pos_dcha(3)=0;
Thrust_dcha=zeros(1,N);
Thrust_dcha(1)=11;
Thrust_dcha(2)=11;
Thrust_dcha(3)=11;
z_dcha=zeros(1,N); z_vel_dcha=zeros(1,N); z_acel_dcha=zeros(1,N);


//***************************//
//DRONE CENTRAL: defino parametros
//CONDICIONES INICIALES DE INESTABILIDAD
//phi mueve en eje Y
//angulotheta mueve en eje X
phi=zeros(1,N);  angulotheta=zeros(1,N); psi=zeros(1,N);
phi_vel= zeros(1,N); angulotheta_vel= zeros(1,N); psi_vel= zeros(1,N); 
phi_acel= 0; angulotheta_acel= 0; psi_acel= 0;

x_vel2=zeros(1,N); x_pos2=zeros(1,N); x_acel2=zeros(1,N);
//x_pos2(1)=120; x_pos2(2)=120; x_pos2(3)=120;
//x_pos2(1)=x_pos_dcha(1)-90; x_pos2(2)=x_pos_dcha(2)-90; x_pos2(3)=x_pos_dcha(3)-90;
x_pos2(1)=x_pos_dcha(1)-60; x_pos2(2)=x_pos_dcha(2)-60; x_pos2(3)=x_pos_dcha(3)-60;
y_acel2=zeros(1,N); y_vel2=zeros(1,N); y_pos=zeros(1,N);
y_pos(1)=0; y_pos(2)=0; y_pos(3)=0; 
z_pos2=zeros(1,N); z_vel2=zeros(1,N); z_acel2=zeros(1,N);
//z_pos2(1)=-64.5; z_pos2(2)=-64.5; z_pos2(3)=-64.5;
Thrust=zeros(1,N);
y_acel=zeros(1,N); y_vel=zeros(1,N);
//**********************************//



//***************************//
//DRON IZQUIERDA: defino parámetros
phi_izq=zeros(1,N); angulotheta_izq=zeros(1,N); psi_izq=zeros(1,N); 
phi_vel_izq=zeros(1,N); angulotheta_vel_izq=zeros(1,N); psi_vel_izq=zeros(1,N);  
z_vel_izq=zeros(1,N); z_pos_izq=zeros(1,N); 
z_acel_izq=zeros(1,N); phi_vel_izq=zeros(1,N); angulotheta_vel_izq=zeros(1,N); psi_vel_izq=zeros(1,N); 
x_acel_izq=zeros(1,N); x_vel_izq=zeros(1,N); x_pos_izq=zeros(1,N); x_vel_izq=zeros(1,N);
//x_pos_izq(1)=x_pos_dcha(1)-180; x_pos_izq(2)=x_pos_dcha(2)-180; x_pos_izq(3)=x_pos_dcha(3)-180;
x_pos_izq(1)=x_pos_dcha(1)-120; x_pos_izq(2)=x_pos_dcha(2)-120; x_pos_izq(3)=x_pos_dcha(3)-120;
y_acel_izq=zeros(1,N); y_vel_izq=zeros(1,N); y_pos_izq=zeros(1,N);
//y_pos_izq(1)=-226.27; y_pos_izq(2)=-226.27; y_pos_izq(3)=-226.27;
//y_pos_izq(1)=-210; y_pos_izq(2)=-210; y_pos_izq(3)=-210;//modificar por aquí
y_pos_izq(1)=0; y_pos_izq(2)=0; y_pos_izq(3)=0;//modificar por aquí



//**********************************///
//4º DRONE
phi4=zeros(1,N);  angulotheta4=zeros(1,N); psi4=zeros(1,N);
phi_vel4= zeros(1,N); angulotheta_vel4= zeros(1,N); psi_vel4= zeros(1,N); 
phi_acel4= 0; angulotheta_acel4= 0; psi_acel4= 0;

x_vel4=zeros(1,N); x_pos4=zeros(1,N); x_acel4=zeros(1,N);
//x_pos2(1)=120; x_pos2(2)=120; x_pos2(3)=120;
x_pos4(1)=x_pos_dcha(1)-270; x_pos4(2)=x_pos_dcha(2)-270; x_pos4(3)=x_pos_dcha(3)-270;
y_acel4=zeros(1,N); y_vel4=zeros(1,N); y_pos4=zeros(1,N);
y_pos4(1)=165; y_pos4(2)=165; y_pos4(3)=165; 
z_pos4=zeros(1,N); z_vel4=zeros(1,N); z_acel4=zeros(1,N);
//z_pos2(1)=-64.5; z_pos2(2)=-64.5; z_pos2(3)=-64.5;
Thrust4=zeros(1,N);



//**********************************///
//5º DRONE
phi5=zeros(1,N);  angulotheta5=zeros(1,N); psi5=zeros(1,N);
phi_vel5= zeros(1,N); angulotheta_vel5= zeros(1,N); psi_vel5= zeros(1,N); 
phi_acel5= 0; angulotheta_acel5= 0; psi_acel5= 0;

x_vel5=zeros(1,N); x_pos5=zeros(1,N); x_acel5=zeros(1,N);
//x_pos2(1)=120; x_pos2(2)=120; x_pos2(3)=120;
x_pos5(1)=x_pos_dcha(1)-360; x_pos5(2)=x_pos_dcha(1)-360; x_pos5(3)=x_pos_dcha(1)-360;
y_acel5=zeros(1,N); y_vel5=zeros(1,N); y_pos5=zeros(1,N);
y_pos5(1)=165; y_pos5(2)=165; y_pos5(3)=165; //por pitagoras
z_pos5=zeros(1,N); z_vel5=zeros(1,N); z_acel5=zeros(1,N);
//z_pos2(1)=-64.5; z_pos2(2)=-64.5; z_pos2(3)=-64.5;
Thrust5=zeros(1,N);


x_vel_deseado=2; 



 //valores del controlador PID
Kp=140.6659; Kd=41.36; Ki=0;
w=0.01;

//Kp=2.008; Kd=4.9687; Ki=0;

xB1=120; xB2=120; xB3=120; xB4=120; //DRONE 3+4
vectorX1=linspace(0,xB1,150);
vectorX2=linspace(0,xB2,150);
vectorX3=linspace(0,xB3,150); //DRONE 3+4
vectorX4=linspace(0,xB4,150); //DRONE 3+4
yB1=0; yB2=0; yB3=0;
z1=zeros(1,length(vectorX1));
z2=zeros(1,length(vectorX2));
z3=zeros(1,length(vectorX3));

matriz_y1=zeros(20,150);
matriz_y2=zeros(20,150);
matriz_y3=zeros(20,150);

        
[y1,x01,y01,c1] = catenaria3(xB1, yB1, Sab, vectorX1);
       
[y2,x02,y02,c2] = catenaria3(xB2, -yB1, Sab, vectorX2);
//vectorX2=vectorX1+120;
[y3,x03,y03,c3] = catenaria3(xB3, -yB1, Sab, vectorX3); //catenaria nueva

[y4,x04,y04,c4] = catenaria3(xB4, yB1, Sab, vectorX4); //catenaria nueva

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
    
    vectorX3=vectorX2+120;
    
     //tensiones verticales de los extremos [kg]
    [TensV_A, TensV_B1, TensH_A, TensH_B1]= TensVerticales(c1,y01,yB1);
    [TensV_B2, TensV_C, TensH_B2, TensH_C]= TensVerticales(c2,y02,yB2);
    
    Thrust(1)=TensV_B1+TensV_B2;
    Thrust(2)=TensV_B1+TensV_B2;
    Thrust(3)=TensV_B1+TensV_B2;
    
    Thrust_izq(1)=TensV_A;
    Thrust_izq(2)=TensV_A;
    Thrust_izq(3)=TensV_A;
    
    Thrust4(1)=TensV_A;
    Thrust4(2)=TensV_A;
    Thrust4(3)=TensV_A;
    
    Thrust5(1)=TensV_A;
    Thrust5(2)=TensV_A;
    Thrust5(3)=TensV_A;
   


//*********************************
//********************************
//*******************************
Matriz_O=zeros(2,1);
matriz_x=zeros(2,1);

Kxd1=0.76; Kxp1=0.22; Kxd2=0.76; Kxp2=0.22; Kxd3=0.76; Kxp3=0.22; Kxd4=0.76; Kxp4=0.22; Kxd5=0.76; Kxp5=0.22;
Kyd1=0.76; Kyp1=0.22; Kyd2=0.76; Kyp2=0.22; Kyd3=0.76; Kyp3=0.22; Kyd4=0.76; Kyp4=0.22; Kyd5=0.76; Kyp5=0.22;
//Kxd=0.1; Kxp=0.1;
//Kyd=0.1; Kyp=0.1;

//matriz_controlador(1)=Kxp;
//matriz_controlador(2)=Kxd;
dist1=zeros(1,N);
dist2=zeros(1,N);
dist3=zeros(1,N);

altura_nudo_central=zeros(1,N);
valores_Kxp1=zeros(1,N); valores_Kxd1=zeros(1,N); valores_Kyp1=zeros(1,N); valores_Kyd1=zeros(1,N);
valores_Kxp2=zeros(1,N); valores_Kxd2=zeros(1,N); valores_Kyp2=zeros(1,N); valores_Kyd2=zeros(1,N);
valores_Kxp3=zeros(1,N); valores_Kxd3=zeros(1,N); valores_Kyp3=zeros(1,N); valores_Kyd3=zeros(1,N);
valores_Kxp4=zeros(1,N); valores_Kxd4=zeros(1,N); valores_Kyp4=zeros(1,N); valores_Kyd4=zeros(1,N);
valores_Kxp5=zeros(1,N); valores_Kxd5=zeros(1,N); valores_Kyp5=zeros(1,N); valores_Kyd5=zeros(1,N);

x_deseado=340;
y_deseado=60;


x_actual=x_pos_dcha(1);
y_actual=y_pos_dcha(1);
x_actual2=x_pos2(1)-90;
y_actual2=y_pos(1);
x_actual3=x_pos_izq(1)-180;
y_actual3=y_pos_izq(1);
x_actual4=x_pos_dcha(1)-270;
y_actual4=y_pos_dcha(1);
x_actual5=y_pos_dcha(1)-360;
y_actual5=y_pos_dcha(1);
y_vel_deseado=2; //por poner algo
yB3=114;
//valores de inicialización
        alpha1 = 0;
        phi_seguidor1 = 0;
        alpha2 = 0;
        phi_seguidor2 = 0;
        rho1=40; rho2=40;
        prevLeaderAngle = 0;
        followerAngle = 0;
//estas variables las he creado para el viento. Tienen que estar inicializadas para la primera iteración
orient1=%pi/2; orient2=%pi/2; orient3=%pi/2; orient4=%pi/2; orient5=%pi/2;

//theta_objetivo=0.2;   
   j=2;
   beta2=%pi/2; beta3=%pi/2;
   while (x_vel_dcha(j) ~= x_vel_deseado)
   
   while (x_pos_dcha(j)~=x_deseado) 
       
       while (y_vel_dcha(j) ~= y_vel_deseado)
   
        while (y_pos_dcha(j)~=y_deseado)
       j=j+1;
        Viento=0;
       
       //DRON LÍDER
       //nuevo
       
        x_deseado=Circuito_x(j);
        y_deseado=Circuito_y(j);
        
       
       
       //y_deseado=x_deseado-240;         
       x_vel_deseado=(x_deseado-x_actual)/(10*dt);
       //x_acel_deseado=(x_deseado-x_actual)/(100*dt*dt);
       x_acel_deseado=(x_vel_deseado-x_vel_dcha(j-1))/(100*dt);
       
        //DIRECCIÓN Y
       y_vel_deseado=(y_deseado-y_actual)/(10*dt);
       //y_acel_deseado=(y_deseado-y_actual)/(100*dt*dt);
       y_acel_deseado=(y_vel_deseado-y_vel_dcha(j-1))/(100*dt);

       theta_objetivo(j)=0.2;
          
            //Kd1=1; Kp1=0.8; //valores muy buenos
                        
            Ux_dcha(j)=x_acel_deseado + Kxd1*(x_vel_deseado - x_vel_dcha(j-1)) + Kxp1*(x_deseado-x_pos_dcha(j-1));
//            
        // a(j)=Ux_dcha(j)*m/(Thrust_dcha(j-1)*cos(angulotheta_dcha(j-1))); así estaba antes
        a(j)=Ux_dcha(j)*m/(Thrust_dcha(j-1)*cos(phi_dcha(j-1)));
        
         //chapuza
        if (a(j))>angulo_max then
            a(j)=angulo_max;

        elseif a(j)<-angulo_max then
            a(j)=-angulo_max;
        end
        theta_objetivo(j)=asin(a(j));
        
        
        Uy_dcha(j)=y_acel_deseado + Kyd1*(y_vel_deseado - y_vel_dcha(j-1)) + Kyp1*(y_deseado-y_pos_dcha(j-1));
        
        a_(j)=Uy_dcha(j)*m/(Thrust_dcha(j-1));
        
        if a_(j)>angulo_max then
            a_(j)=angulo_max;
        elseif a_(j)<-angulo_max then
            a_(j)=-angulo_max;
        end
        phi_objetivo(j)=-asin(a_(j));
        
       
    
    //MUEVO EL DRON DCHA
    //*********************************
    //calculo tension variable TensV_A
    xB1=sqrt((x_pos_dcha(j-1)-x_pos2(j-2))^2 + (y_pos_dcha(j-1)-y_pos(j-2))^2);
    yB1=descenso(Sab, xB1,w);

    altura_nudo_central(j)=yB1;
    
    vectorX1=linspace(0,xB1,150);
    
    //PRIMER TRAMO: Catenaria o Spline según distancia
    if ((xB1<75)|(xB1>165))
        //Cubic Spline
        disp("Cubic spline");
        dist1(j)=xB1;
    else
        [y1,x01,y01,c1] = catenaria3(xB1, -yB1, Sab, vectorX1);
        dist1(j)=xB1;
        [TensV_A, TensV_B1, TensH_A, TensH_B1]= TensVerticales(c1,y01,yB1);
    //plot(sqrt((x_pos_dcha(j)-x_pos2(j-1))^2 - (y_pos_dcha(j)-y_pos(j-1))^2),'or'); //sirve para ver la distancia real de los hilos
end
    
    [z_acel_dcha(j), phi_dcha(j), angulotheta_dcha(j), psi_dcha(j), phi_vel_dcha(j), angulotheta_vel_dcha(j), psi_vel_dcha(j), Thrust_dcha(j)] = quadrotor_dcha2(phi_dcha(j-1), angulotheta_dcha(j-1), psi_dcha(j-1), phi_vel_dcha(j-1),phi_vel_dcha(j-2), angulotheta_vel_dcha(j-1),angulotheta_vel_dcha(j-2), psi_vel_dcha(j-1),psi_vel_dcha(j-2), 0, TensV_A, z_vel_dcha(j-1),z_vel_dcha(j-2),z_dcha(j-1), TensH_B1, 0,theta_objetivo(j),phi_objetivo(j-1),Viento,orient1);
    
//    Thrust_dcha(j)=Thrust_dcha(j)+(sin(orient1)*sin(angulotheta_dcha(j))*cos(phi_dcha(j))-cos(orient1)*sin(phi_dcha(j)))*Viento;
    if abs(Thrust_dcha(j))>20
        Thrust_dcha(j)=20;
    end
    

    
    x_acel_dcha(j)=(cos(psi_dcha(j))*cos(angulotheta_dcha(j)))*(TensH_B1)/m + (sin(psi_dcha(j))*sin(phi_dcha(j))+cos(psi_dcha(j))*sin(angulotheta_dcha(j))*cos(phi_dcha(j)))*Thrust_dcha(j)/m + Viento*cos(orient1);
 
    x_vel_dcha(j)=x_vel_dcha(j-1)+ x_acel_dcha(j)/2*dt;
    //x_vel_dcha(j)=x_vel_dcha(j-1)+(x_acel_dcha(j)-x_acel_dcha(j-1))*dt;
    x_pos_dcha(j)=x_pos_dcha(j-1)+ x_vel_dcha(j)/2*dt;
    x_actual=x_pos_dcha(j);
    
    //muevo en Y
    y_acel_dcha(j)=(sin(psi_dcha(j))*sin(angulotheta_dcha(j))*cos(phi_dcha(j))-cos(psi_dcha(j))*sin(phi_dcha(j)))*Thrust_dcha(j)/m - Viento*sin(orient1);
    
    y_vel_dcha(j)=y_vel_dcha(j-1)+ y_acel_dcha(j)/2*dt;
    
    y_pos_dcha(j)=y_pos_dcha(j-1)+ y_vel_dcha(j)/2*dt;
    y_actual=y_pos_dcha(j);
    
    //orientación drone
    orient1=atan((y_pos_dcha(j)-y_pos_dcha(j-1)),(x_pos_dcha(j)-x_pos_dcha(j-1))); 
    
       
    
    if j>4 then
            
      P_ex= (theta_objetivo(j)-angulotheta_dcha(j))/(theta_objetivo(j))*100;
      if abs(P_ex)> 1 & abs(P_ex)<=3
          mu=0.5*P_ex-0.5;
          Kxd1=abs(Kxd1 + 0.5*mu*(theta_objetivo(j)-angulotheta_dcha(j)));
      elseif abs(P_ex)>3 & abs(P_ex)<=5
          mu=-0.5*P_ex+2.5;
          Kxd1=abs(Kxd1 + 0.5*mu*(theta_objetivo(j)-angulotheta_dcha(j)));
      elseif abs(P_ex)>4 & abs(P_ex)<=6.5
          mu=(P_ex-4)/2.5;
          Kxp1=abs(Kxp1 + 0.5*mu*(theta_objetivo(j)-angulotheta_dcha(j)));
      elseif abs(P_ex)>6.5 & abs(P_ex)<=9
          mu=(9-P_ex)/2.5;
          Kxp1=abs(Kxp1 + 0.5*mu*(theta_objetivo(j)-angulotheta_dcha(j)));        
      end
end


if j>4 then
    P_ey=(phi_objetivo(j)-phi_dcha(j))/(phi_objetivo(j))*100;

         if abs(P_ey)> 1 & abs(P_ey)<=3
          mu=0.5*P_ey-0.5;
          Kyd1=abs(Kyd1 + 0.5*mu*(phi_objetivo(j)-phi_dcha(j)));
      elseif abs(P_ey)>3 & abs(P_ey)<=5
          mu=-0.5*P_ey+2.5;
          Kyd1=abs(Kyd1 + 0.5*mu*(phi_objetivo(j)-phi_dcha(j)));
      elseif abs(P_ey)>4 & abs(P_ey)<=6.5
          mu=(P_ey-4)/2.5;
          Kyp1=abs(Kyp1 + 0.5*mu*(phi_objetivo(j)-phi_dcha(j)));
      elseif abs(P_ey)>6.5 & abs(P_ey)<=9
          mu=(9-P_ey)/2.5;
          Kyp1=abs(Kyp1 + 0.5*mu*(phi_objetivo(j)-phi_dcha(j)));        
      end
end

valores_Kxp1(j)=Kxp1;
valores_Kxd1(j)=Kxd1;
valores_Kyp1(j)=Kyp1;
valores_Kyd1(j)=Kyd1;


disp(j);

    
    //*********************************
    //LE SIGUE EL DRON CENTRAL
    
//    x_deseado2=x_pos_dcha(j-1)-120; //si pongo x_deseado=x_pos_dcha(j-1)-120 -> el angulo tetha me sale muy irregular
//    y_deseado2=y_deseado; //esto lo pongo para inicializar
//nuevo
x_deseado2=x_pos_dcha(j);
y_deseado2=y_pos_dcha(j);
   
    //platoon
    if j>4
         //funcion chuck domingo
        [x_deseado2,y_deseado2,alpha1,phi_seguidor1,rho1]=seguidores(x_pos_dcha(j),x_pos2(j-1),y_pos_dcha(j),y_pos(j-1),x_pos_dcha(j-1),y_pos_dcha(j-1),alpha1, phi_seguidor1,rho1);
        
        plot(x_deseado2,y_deseado2,'or');
        
    end

    x_vel_deseado2=(x_deseado2-x_actual2)/(10*dt);
    x_acel_deseado2=(x_vel_deseado2-x_vel2(j-1))/(100*dt);
    y_vel_deseado2=(y_deseado2-y_actual2)/(10*dt);
   // y_acel_deseado2=(y_deseado2-y_actual2)/(100*dt*dt);
   y_acel_deseado2=(y_vel_deseado2-y_vel(j-1))/(100*dt);
       


//theta_objetivo=0.2;   
    //CALCULO TENSIONES VARIABLES POR ESTAR MOVIÉNDOSE LOS DRONES
   xB2=sqrt((x_pos2(j-1)-x_pos_izq(j-2))^2 + (y_pos(j-1)-y_pos_izq(j-2))^2);
   //yB2=abs(yB1)-abs(yB3);
   yB2=descenso(Sab,xB2,w); //ponia 200
   vectorX2=linspace(0, xB2, 150);
   
   //SEGUNDO TRAMO: Catenaria o Spline según distancia
   if ((xB2<75)|(xB2>150))
       //Cubic Spline
        dist2(j)=xB2;
       disp("segunda cubic");
   else
       //catenaria
       [y2,x02,y02,c2] = catenaria3(xB2, yB1, Sab, vectorX2); 
       dist2(j)=xB2;
       [TensV_B2, TensV_C, TensH_B2, TensH_C]= TensVerticales(c2,y02,yB2);
       end

       
       theta_objetivo2(j)=0.2;
                //Kd1=1; Kp1=0.8; //valores muy buenos
                Ux(j)=x_acel_deseado2 + Kxd2*(x_vel_deseado2 - x_vel2(j-1)) + Kxp2*(x_deseado2-x_pos2(j-1));
//         
         a2(j)=Ux(j)*m/(Thrust(j-1)*cos(angulotheta(j-1)));
         //chapuza
        if a2(j)>angulo_max then
            a2(j)=angulo_max;
        elseif a2(j)<-angulo_max then
            a2(j)=-angulo_max;
        end
        theta_objetivo2(j)=asin(a2(j));
        
        
        Uy(j)=y_acel_deseado2 + Kyd2*(y_vel_deseado2 - y_vel(j-1)) + Kyp2*(y_deseado2-y_pos(j-1));
        
        a_2(j)=Uy(j)*m/(Thrust(j-1));
        
        if a_2(j)>angulo_max then
            a_2(j)=angulo_max;
        elseif a_2(j)<-angulo_max then
            a_2(j)=-angulo_max;
        end
        phi_objetivo2(j)=-asin(a_2(j));
              
         
    [z_acel2(j), phi(j), angulotheta(j), psi(j), phi_vel(j), angulotheta_vel(j), psi_vel(j), Thrust(j)] = quadrotor_dcha2(phi(j-1), angulotheta(j-1), psi(j-1), phi_vel(j-1),phi_vel(j-2), angulotheta_vel(j-1),angulotheta_vel(j-2), psi_vel(j-1),psi_vel(j-2), 0, TensV_B2+TensV_B1, z_vel2(j-1),z_vel2(j-2),z_pos2(j-1), TensH_B1, TensH_B2 ,theta_objetivo2(j),phi_objetivo2(j),Viento,orient2);
    
//    Thrust(j)=Thrust(j)+(sin(orient2)*sin(angulotheta(j))*cos(phi_dcha(j))-cos(orient2)*sin(phi_dcha(j)))*Viento;
    if Thrust(j)>20
        Thrust(j)=20;
    end
    

    x_acel2(j)=(cos(psi(j))*cos(angulotheta(j)))*(TensH_B1-TensH_B2)/m + (sin(psi(j))*sin(phi(j))+cos(psi(j))*sin(angulotheta(j))*cos(phi(j)))*Thrust(j)/m + Viento*cos(orient2);
 
    x_vel2(j)=x_vel2(j-1)+ x_acel2(j)/2*dt;
    //x_vel_dcha(j)=x_vel_dcha(j-1)+(x_acel_dcha(j)-x_acel_dcha(j-1))*dt;
    x_pos2(j)=x_pos2(j-1)+ x_vel2(j)/2*dt;
    x_actual2=x_pos2(j);
    
    distancia1(j)=x_pos_dcha(j)-x_pos2(j); //distancia entre el dron derecho y el central. Me sirve para calcular la tensión de la catenaria.
    
    //DIRECCIÓN Y
     y_acel(j)=(sin(psi(j))*sin(angulotheta(j))*cos(phi(j))-cos(psi(j))*sin(phi(j)))*Thrust(j)/m - Viento*sin(orient2);
    y_vel(j)=y_vel(j-1)+ y_acel(j)/2*dt;
    y_pos(j)=y_pos(j-1)+ y_vel(j)/2*dt;
    y_actual2=y_pos(j);
    
    //orientación drone
    orient2=atan((y_pos(j)-y_pos(j-1))/(x_pos2(j)-x_pos2(j-1)));
    
    
       if j>4 then
      P_ex= (theta_objetivo2(j)-angulotheta(j))/(theta_objetivo2(j))*100;
        
      if abs(P_ex)> 1 & abs(P_ex)<=3
          mu=0.5*P_ex-0.5;
          Kxd2=abs(Kxd1 + 0.5*mu*(theta_objetivo2(j)-angulotheta(j)));
      elseif abs(P_ex)>3 & abs(P_ex)<=5
          mu=-0.5*P_ex+2.5;
          Kxd2=abs(Kxd1 + 0.5*mu*(theta_objetivo2(j)-angulotheta(j)));
      elseif abs(P_ex)>4 & abs(P_ex)<=6.5
          mu=(P_ex-4)/2.5;
          Kxp2=abs(Kxp1 + 0.5*mu*(theta_objetivo2(j)-angulotheta(j)));
      elseif abs(P_ex)>6.5 & abs(P_ex)<=9
          mu=(9-P_ex)/2.5;
          Kxp2=abs(Kxp1 + 0.5*mu*(theta_objetivo2(j)-angulotheta(j)));        
      end
end

if j>4 then
    P_ey=(phi_objetivo2(j)-phi(j))/(phi_objetivo2(j))*100;
     if abs(P_ey)> 1 & abs(P_ey)<=3
          mu=0.5*P_ey-0.5;
          Kyd2=abs(Kyd1 + 0.5*mu*(phi_objetivo2(j)-phi(j)));
      elseif abs(P_ey)>3 & abs(P_ey)<=5
          mu=-0.5*P_ey+2.5;
          Kyd2=abs(Kyd1 + 0.5*mu*(phi_objetivo2(j)-phi(j)));
      elseif abs(P_ey)>4 & abs(P_ey)<=6.5
          mu=(P_ey-4)/2.5;
          Kyp2=abs(Kyp1 + 0.5*mu*(phi_objetivo2(j)-phi(j)));
      elseif abs(P_ey)>6.5 & abs(P_ey)<=9
          mu=(9-P_ey)/2.5;
          Kyp2=abs(Kyp1 + 0.5*mu*(phi_objetivo2(j)-phi(j)));        
      end
   // disp(P_ex,P_ey);
end

valores_Kxp2(j)=Kxp2;
valores_Kxd2(j)=Kxd2;
valores_Kyp2(j)=Kyp2;
valores_Kyd2(j)=Kyd2;


    
    //LE SIGUE EL DRON IZQUIERDO
    //************************************
    
    x_deseado3=x_pos2(j);
    y_deseado3=y_pos(j);
       
       
    if j>4
        [x_deseado3,y_deseado3,alpha2,phi_seguidor2,rho2]=seguidores(x_pos2(j),x_pos_izq(j-1),y_pos(j),y_pos_izq(j-1),x_pos2(j-1),y_pos(j-1),alpha2,phi_seguidor2,rho2);
        //calculo el ángulo que forman el drone líder y el siguiente
    end


       
    x_vel_deseado3=(x_deseado3-x_actual3)/(10*dt);
    //x_acel_deseado3=(x_deseado3-x_actual3)/(100*dt*dt);
    x_acel_deseado3=(x_vel_deseado3-x_vel_izq(j-1))/(100*dt);
    y_vel_deseado3=(y_deseado3-y_actual3)/(10*dt);
    //y_acel_deseado3=(y_deseado3-y_actual3)/(100*dt*dt);
    y_acel_deseado3=(y_vel_deseado3-y_vel_izq(j-1))/(100*dt);
 
       
       theta_objetivo3(j)=0.2;
          
            //Kd1=1; Kp1=0.8; //valores muy buenos
            Ux_izq(j)=x_acel_deseado3 + Kxd3*(x_vel_deseado3 - x_vel_izq(j-1)) + Kxp3*(x_deseado3-x_pos_izq(j-1));

         a3(j)=Ux_izq(j)*m/(Thrust_izq(j-1)*cos(angulotheta(j-1)));
         //chapuza
        if a3(j)>angulo_max then
            a3(j)=angulo_max;
        elseif a3(j)<-angulo_max then
            a3(j)=-angulo_max;
        end
        theta_objetivo3(j)=asin(a3(j));
        
        
        //DIRECCIÓN Y

       
        Uy_izq(j)=y_acel_deseado3 + Kyd3*(y_vel_deseado3 - y_vel_izq(j-1)) + Kyp3*(y_deseado3-y_pos_izq(j-1));
        
        a_3(j)=Uy_izq(j)*m/(Thrust_izq(j-1));
        
        if a_3(j)>angulo_max then
            a_3(j)=angulo_max;
        elseif a_3(j)<-angulo_max then
            a_3(j)=-angulo_max;
        end
        phi_objetivo3(j)=-asin(a_3(j));
        
        
    
    [z_acel_izq(j), phi_izq(j), angulotheta_izq(j), psi_izq(j), phi_vel_izq(j), angulotheta_vel_izq(j), psi_vel_izq(j), Thrust_izq(j)] = quadrotor_dcha2(phi_izq(j-1), angulotheta_izq(j-1), psi_izq(j-1), phi_vel_izq(j-1),phi_vel_izq(j-2), angulotheta_vel_izq(j-1),angulotheta_vel_izq(j-2), psi_vel_izq(j-1),psi_vel_izq(j-2), 0, TensV_B2+TensV_B1, z_vel_izq(j-1),z_vel_izq(j-2),z_pos_izq(j-1), 0, TensH_B2 ,theta_objetivo3(j),phi_objetivo3(j),Viento,orient3);
    
//    Thrust_izq(j)=Thrust_izq(j)+(sin(orient3)*sin(angulotheta_izq(j))*cos(phi_izq(j))-cos(orient3)*sin(phi_izq(j)))*Viento;
    if Thrust_izq(j)>20
        Thrust_izq(j)=20;
    end
    

    x_acel_izq(j)=(cos(psi_izq(j))*cos(angulotheta_izq(j)))*(-TensH_B2)/m + (sin(psi_izq(j))*sin(phi_izq(j))+cos(psi_izq(j))*sin(angulotheta_izq(j))*cos(phi_izq(j)))*Thrust_izq(j)/m + Viento*cos(orient3);
 
    x_vel_izq(j)=x_vel_izq(j-1)+ x_acel_izq(j)/2*dt;
    //x_vel_dcha(j)=x_vel_dcha(j-1)+(x_acel_dcha(j)-x_acel_dcha(j-1))*dt;
    x_pos_izq(j)=x_pos_izq(j-1)+ x_vel_izq(j)/2*dt;
    x_actual3=x_pos_izq(j);
    distancia2(j)=x_pos2(j)-x_pos_izq(j); //distancia entre el dron central y el izquierdo. Me sirve para calcular la tensión vertical de las catenarias.
    
     y_acel_izq(j)=(sin(psi_izq(j))*sin(angulotheta_izq(j))*cos(phi_izq(j))-cos(psi_izq(j))*sin(phi_izq(j)))*Thrust_izq(j)/m - Viento*sin(orient3);
    y_vel_izq(j)=y_vel_izq(j-1)+ y_acel_izq(j)/2*dt;
    y_pos_izq(j)=y_pos_izq(j-1)+ y_vel_izq(j)/2*dt;
    y_actual3=y_pos_izq(j);
    

    //orientación drone
    orient3=atan((y_pos_izq(j)-y_pos_izq(j-1))/(x_pos_izq(j)-x_pos_izq(j-1)));
    
    
if j>4 then
    
        //calculo beta3
       if (x_pos_izq(j)-x_pos_izq(j-1))<0
           if (y_pos_izq(j)-y_pos_izq(j-1))>0
               beta3=%pi - atan(abs((y_pos_izq(j)-y_pos_izq(j-1))/(x_pos_izq(j)-x_pos_izq(j-1))));
           else
               beta3=%pi + atan(abs((y_pos_izq(j)-y_pos_izq(j-1))/(x_pos_izq(j)-x_pos_izq(j-1))));
           end
       else
           if (y_pos_izq(j)-y_pos_izq(j-1))>0
               beta3=atan((y_pos_izq(j)-y_pos_izq(j-1))/(x_pos_izq(j)-x_pos_izq(j-1)));
           else
               beta3=atan((y_pos_izq(j)-y_pos_izq(j-1))/(x_pos_izq(j)-x_pos_izq(j-1)));
           end
       end

        P_ex= (theta_objetivo3(j)-angulotheta_izq(j))/(theta_objetivo3(j))*100;
      if abs(P_ex)> 1 & abs(P_ex)<=3
          mu=0.5*P_ex-0.5;
          Kxd3=abs(Kxd3 + 0.5*mu*(theta_objetivo3(j)-angulotheta_izq(j)));
      elseif abs(P_ex)>3 & abs(P_ex)<=5
          mu=-0.5*P_ex+2.5;
          Kxd3=abs(Kxd3 + 0.5*mu*(theta_objetivo3(j)-angulotheta_izq(j)));
      elseif abs(P_ex)>4 & abs(P_ex)<=6.5
          mu=(P_ex-4)/2.5;
          Kxp3=abs(Kxp3 + 0.5*mu*(theta_objetivo3(j)-angulotheta_izq(j)));
      elseif abs(P_ex)>6.5 & abs(P_ex)<=9
          mu=(9-P_ex)/2.5;
          Kxp3=abs(Kxp3 + 0.5*mu*(theta_objetivo3(j)-angulotheta_izq(j)));        
      end
end

if j>4 then
    P_ey=(phi_objetivo3(j)-phi_izq(j))/(phi_objetivo3(j))*100;
    if abs(P_ey)> 1 & abs(P_ey)<=3
          mu=0.5*P_ey-0.5;
          Kyd3=abs(Kyd3 + 0.5*mu*(phi_objetivo3(j)-phi_izq(j)));
      elseif abs(P_ey)>3 & abs(P_ey)<=5
          mu=-0.5*P_ey+2.5;
          Kyd3=abs(Kyd3 + 0.5*mu*(phi_objetivo3(j)-phi_izq(j)));
      elseif abs(P_ey)>4 & abs(P_ey)<=6.5
          mu=(P_ey-4)/2.5;
          Kyp3=abs(Kyp3 + 0.5*mu*(phi_objetivo3(j)-phi_izq(j)));
      elseif abs(P_ey)>6.5 & abs(P_ey)<=9
          mu=(9-P_ey)/2.5;
          Kyp3=abs(Kyp3 + 0.5*mu*(phi_objetivo3(j)-phi_izq(j)));        
      end
end

valores_Kxp3(j)=Kxp3;
valores_Kxd3(j)=Kxd3;
valores_Kyp3(j)=Kyp3;
valores_Kyd3(j)=Kyd3;

        end
end
end
end


