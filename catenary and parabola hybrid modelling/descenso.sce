//funcion para hallar el equilibrio vertical

function [Y2] = descenso(L0,Lx,w)


Y2=-40; //Valor para empezar a iterar
expresion_A=sqrt(3)*sqrt((L0^2 - Y2^2)/(Lx^2)-1);

iter2=1;

F1_1=w/2*(-Y2*coth(expresion_A)+L0)-2*w*L0/3;
//F2_1=w/2*(-coth(sqrt(3*((L0^2-Y2^2)/(Lx)^2)-1))-(sqrt(3)*Y2^2*(csch(sqrt(3*((L0^2-Y2^2)/(Lx)^2)-1)))^2/((Lx)^2*sqrt(3*((L0^2-Y2^2)/(Lx)^2)-1))));
F2_1=w/2*(-coth(expresion_A)-(sqrt(3)*Y2^2*(csch(expresion_A))^2/((Lx^2)*expresion_A/sqrt(3)))); 

Y2=Y2-F1_1/F2_1;
F3_1=w/2*(-Y2*coth(expresion_A)+L0)-2*w*L0/3;


//la precision de millonésimas es importante. Probe con Wolfram Alpha y daba un poco mejor que esta precisión

while(abs(F1_1-F3_1)>0.0000000001 & abs(F1_1)>0.0001) then
   
    //función que dice que un extremo tiene que cargar la 1/3 del peso total de 2 catenarias
    F1_1=w/2*(-Y2*coth(expresion_A)+L0)-2*w*L0/3; 
   
    //derivada de la función de F1_1
    F2_1=w/2*(-coth(expresion_A)-(sqrt(3)*Y2^2*(csch(expresion_A))^2/((Lx^2)*expresion_A/sqrt(3))));  
    
    //ESTO ES LA FÓRMULA DE CALCULAR NUEVOS VALORES PARA NEWTON-RAPHSON
    Y2=Y2-F1_1/F2_1;
    F3_1=w/2*(-Y2*coth(expresion_A)+L0)-2*w*L0/3;
    iter2=iter2+1;
end 

endfunction

// programación en Wolfram Alpha
//  w/2*(-X*coth(sqrt(3)*sqrt((70^2-X^2)/(50)^2-1))+70)-2*w*70/3 =0
//http://programming-with-scilab.blogspot.com.es/2012/04/solution-of-single-non-linear-equation.html
//hay que ser más preciso
