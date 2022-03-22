//las tensiones las calculo en kg
function [TensV_A, TensV_B, TensH_A, TensH_B]=TensVerticales(c,y0,Y2)

//5 mg por cm    
//w=0.005; //kg por unidad de longitud de la catenaria. Esta en [kg/cm]
//VOY A CONSIDERAR QUE A ES EL EXTREMO IZQUIERDO Y B EL DERECHO
T0=c*w;
T_A=w*(abs(y0));
ang_a=acos(T0/T_A);

//Tension_V_B1=T_B1*sin(ang_b1);
TensV_A=T_A*sin(ang_a);
TensH_A=T_A*cos(ang_a);


//Tension del punto B (Primera catenaria)
//T02=c2*w;
T_B=w*(abs(y0)+Y2);
ang_b=abs(acos(T0/T_B));
TensV_B=T_B*sin(ang_b);
TensH_B=T_B*cos(ang_b);

endfunction
