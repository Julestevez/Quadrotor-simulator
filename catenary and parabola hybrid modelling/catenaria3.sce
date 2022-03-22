//catenaria

function [y,x0,y0,c] = catenaria3(xB, yB, Sab, vectorX)


// elijo que la separación entre ellos sea de 20m (yA a 50; yB a 70m)
// separados por xB (60:0.1:80)

// en la ecuación tengo x0,y0 y c que hay que hallarlos

//Sab= 140; //longitud de la cuerda

// xB=55.42;

// yB= 20;
c=3;

f1=sqrt(Sab^2 - yB^2)/xB;
f2=sinh(xB/(2*c))/xB*(2*c);


while (abs(f2-f1)>0.01)
    c=c+0.01;
    f1=sqrt(Sab^2 - yB^2)/xB;
    f2=sinh(xB/(2*c))/xB*(2*c);
end



// ahora consigo la x0, y0
y0=c;

//consigo x0
x0=3;

f3= c*cosh((xB-x0)/c) - c*cosh(x0/c);

while(abs(f3-yB)>0.1)
    x0=x0+0.01;
    f3= c*cosh((xB-x0)/c) - c*cosh(x0/c);
end

//hallo y0
y0 = yB - c*cosh((xB-x0)/c);


//dibujo la catenaria
y=y0 + c*cosh((vectorX-x0)/c);


endfunction
