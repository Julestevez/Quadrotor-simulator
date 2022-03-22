//este cuadricóptero tiene configuración +. Es decir, un rotor apuntando a cada dirección.
//PHI=ángulo de izquierda-derecha (roll)
//ANGULOTHETA=hacia adelante-atrás (pitch)
//PSI=rotación sobre eje z (yaw)

function [z_acel, phi, angulotheta, psi, phi_vel, angulotheta_vel, psi_vel, Thrust] = quadrotor_dcha2(phi_ant, angulotheta_ant, psi_ant, phi_vel_ant, phi_vel_2ant, angulotheta_vel_ant, angulotheta_vel_2ant, psi_vel_ant, psi_vel_2ant, Ymax, catenary_load, z_vel_ant, z_vel_2ant, z_ant, TensH_A, TensH_B,theta_objetivo,phi_objetivo);           
    
    g=9.80; 
    //parece que el fallo está aquí. No son velocidades ni alturas, sino diferencias entre un estado y el anterior      
//    global Torque(2);
    //CONTROLADOR PID: ROLL=0
    //phi_objetivo=0;
    angulophi_objetivo=phi_objetivo;
    err_phi=phi_objetivo -phi_ant;
    Torque(2)=(err_phi*Kp + err_phi*Ki*dif_tiempo - (phi_vel_ant-phi_vel_2ant)*Kd*dif_tiempo )* I(1,1);
    
    
    //CONTROLADOR PID: PITCH=0
    //angulotheta_objetivo=0;
    angulotheta_objetivo=theta_objetivo;
    err_theta=angulotheta_objetivo -angulotheta_ant;
    Torque(3)=(err_theta*Kp + err_theta*Ki*dif_tiempo - (angulotheta_vel_ant-angulotheta_vel_2ant)*Kd*dif_tiempo) * I(2,2);
    
    
    //CONTROLADOR PID: YAW=0
    psi_objetivo=-psi_ant;
    err_yaw=psi_objetivo -psi_ant;
    Torque(4)=(err_yaw*Kp + err_yaw*Ki*dif_tiempo - (psi_vel_ant-psi_vel_2ant)*Kd*dif_tiempo ) * I(3,3);
    
    //disp(TensH_A);
    //CONTROLADOR DE ALTURA
    err_altura=-Ymax + z_ant;
    Torque(1)=err_altura*Kp + err_altura*Ki*dif_tiempo - (z_vel_ant-z_vel_2ant)*Kd*dif_tiempo + (m*g+catenary_load*g)/(cos(phi_ant)*cos(angulotheta_ant)) - sin(angulotheta_ant)*(TensH_A + TensH_B)*g;
    Thrust=Torque(1);
    
     if Thrust>20
        Thrust=20;
    end
    if Thrust<5
       Thrust=5;
   end
    
          
    //calculo los inputs
    inputs(1)= 1/(4*k)*Torque(1) - 1/(2*k*L)*Torque(3) - 1/(4*b)*Torque(4);
    inputs(2)= 1/(4*k)*Torque(1) - 1/(2*k*L)*Torque(2) + 1/(4*b)*Torque(4);
    inputs(3)= 1/(4*k)*Torque(1) + 1/(2*k*L)*Torque(3) - 1/(4*b)*Torque(4);
    inputs(4)= 1/(4*k)*Torque(1) + 1/(2*k*L)*Torque(2) + 1/(4*b)*Torque(4);
    
    //disp(Torque)
//    plot(j,Torque(1),'ob');
//    plot(j,sqrt(inputs(1)),'ob');
//    plot(j,sqrt(inputs(2)),'or');
//    plot(j,sqrt(inputs(3)),'og');
//    plot(j,sqrt(inputs(4)),'*b');
//    
    phi_acel=         Torque(2)/I(1,1);
    angulotheta_acel= Torque(3)/I(2,2);
    psi_acel=         Torque(4)/I(3,3);
    
    phi_vel=         phi_vel_ant + phi_acel*dt;
    angulotheta_vel= angulotheta_vel_ant + angulotheta_acel*dt;
    psi_vel=         psi_vel_ant + psi_acel*dt;
    
    phi=         phi_ant + phi_vel*dt;
    angulotheta= angulotheta_ant + angulotheta_vel*dt;
    psi=         psi_ant + psi_vel*dt;
    
    z_acel= -g - catenary_load*g/m + sin(angulotheta_ant)*(TensH_B)/m + (cos(angulotheta_ant)*cos(phi_ant))*Torque(1)/m; //[N]/m
    phi=0.97*(phi_ant)+0.03*phi_acel*dt;
angulotheta= 0.97*(angulotheta_ant)+0.03*angulotheta_acel*dt;
    
endfunction

       
