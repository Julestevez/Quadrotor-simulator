function [x_acel,x_vel,x_pos,x_actual,y_acel,y_vel,y_pos,y_actual]=AcelVelPos(psi,angulotheta,phi,TensH_B1,Thrust,Viento,orient,x_acel,x_vel,x_pos,y_acel,y_vel,y_pos)
    x_acel=(cos(psi)*cos(angulotheta))*(TensH_B1)/m + (sin(psi)*sin(phi)+cos(psi)*sin(angulotheta)*cos(phi))*Thrust/m + Viento*cos(orient);
 
    x_vel=x_vel+ x_acel/2*dt;
    //x_vel_dcha(j)=x_vel_dcha(j-1)+(x_acel_dcha(j)-x_acel_dcha(j-1))*dt;
    x_pos=x_pos+ x_vel/2*dt;
    x_actual=x_pos;
    
    //muevo en Y
    y_acel=(sin(psi)*sin(angulotheta)*cos(phi)-cos(psi)*sin(phi))*Thrust/m - Viento*sin(orient);
    
    y_vel=y_vel+ y_acel/2*dt;
    
    y_pos=y_pos+ y_vel/2*dt;
    y_actual=y_pos;
endfunction
