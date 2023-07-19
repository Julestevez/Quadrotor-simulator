#catenary calculus function
import numpy as np
import math


def catenary(HorDif,VerDif,S,VectorX):

    #calculus of x0 and y0 (see the reference web)

    c=3 #initialization

    f1=math.sqrt(S**2 - VerDif**2)/HorDif
    f2=math.sinh(HorDif/(2*c))/HorDif*(2*c)


    while (abs(f2-f1)>0.01):
        c=c+0.01
        f1=math.sqrt(S**2 - VerDif**2)/HorDif
        f2=math.sinh(HorDif/(2*c))/HorDif*(2*c)


    x0=3 #initialization

    f3= c*math.cosh((HorDif-x0)/c) - c*math.cosh(x0/c)

    while(abs(f3-VerDif)>0.1):
        x0=x0+0.01
        f3= c*math.cosh((HorDif-x0)/c) - c*math.cosh(x0/c)


    y0 = VerDif - c*math.cosh((HorDif-x0)/c)

    #generate the y points of the catenary
    fun = lambda vecX : y0 + c * math.cosh((vecX - x0) / c)
    y= list(map(fun, VectorX)) #it's a list
    y=np.asarray(y)

    return y,x0,y0,c