// anim_block_rotate.sce
// ejemplo extraído de http://www.sky-engin.jp/en/ScilabAnimation/chap10/chap10.html


clc; xdel(winsid());

exec('Euler2R.sci',-1);
exec('GeoVerMakeBlock.sci',-1);
exec('GeoPatMakeBlock.sci',-1);

function anim_block2(A,r,n_time,A1,r1,A2,r2,vectorX1,matriz_y1,vectorX2,matriz_y2,x_pos2,y_pos2)
    
    //x_pos2 es lo que se mueve en la dirección X
    
    vector_ceros=zeros(1,150); //en principio no uso esto
    
    vector_auxiliar=ones(1,150);
    vector_auxiliar2=vector_auxiliar*x_pos2/100;
    vector_auxiliar3=vector_auxiliar*y_pos2/100;
     

 
// Block specification
Lx = 0.15;
Ly = 0.15;
Lz = 0.05;
n_time=100;

// Motion data
//t = [0:0.005:1]';        // Time data
//r = [0*t, 0*t, 0.8*t];     // Position data
//A = [2*%pi*t, 0*t, 0*t]; // Orientation data (x-y-z Euler angle)


//r1 = [0*t, 0.5*t, 0*t];     // Position data
//A1 = [0*t, 2*%pi*t, 0*t]; // Orientation data (x-y-z Euler angle)
//
//n_time = length(t);

// Compute propagation of vertices and patches
for i_time=1:n_time

    R = Euler2R(A(i_time,:));
    VertexData(:,:,i_time) = GeoVerMakeBlock(r(i_time,:),R,[Lx,Ly,Lz]);
    [X,Y,Z] = GeoPatMakeBlock(VertexData(:,:,i_time));
    PatchData_X(:,:,i_time) = X;
    PatchData_Y(:,:,i_time) = Y;
    PatchData_Z(:,:,i_time) = Z;
    
    R1 = Euler2R(A1(i_time,:));
    VertexData1(:,:,i_time) = GeoVerMakeBlock(r1(i_time,:),R1,[Lx,Ly,Lz]);
    [X1,Y1,Z1] = GeoPatMakeBlock(VertexData1(:,:,i_time));
    PatchData_X1(:,:,i_time) = X1;
    PatchData_Y1(:,:,i_time) = Y1;
    PatchData_Z1(:,:,i_time) = Z1;
    
    R2 = Euler2R(A2(i_time,:));
    VertexData2(:,:,i_time) = GeoVerMakeBlock(r2(i_time,:),R2,[Lx,Ly,Lz]);
    [X2,Y2,Z2] = GeoPatMakeBlock(VertexData2(:,:,i_time));
    PatchData_X2(:,:,i_time) = X2;
    PatchData_Y2(:,:,i_time) = Y2;
    PatchData_Z2(:,:,i_time) = Z2;
end

// Draw initial figure
figure(1);
drawlater();
plot3d(PatchData_X(:,:,1),PatchData_Y(:,:,1),PatchData_Z(:,:,1));
h_fac3d = gce();
h_fac3d.color_mode = 4;
h_fac3d.foreground = 1;
h_fac3d.hiddencolor = 4;

plot3d(PatchData_X1(:,:,1),PatchData_Y1(:,:,1),PatchData_Z1(:,:,1));
h_fac3d1=gce();
h_fac3d1.color_mode = 5;
h_fac3d1.foreground = 1;
h_fac3d1.hiddencolor = 4;

plot3d(PatchData_X2(:,:,1),PatchData_Y2(:,:,1),PatchData_Z2(:,:,1));
h_fac3d2 = gce();
h_fac3d2.color_mode = 3;
h_fac3d2.foreground = 1;
h_fac3d2.hiddencolor = 4;
drawnow();


// Axes settings
xlabel("x",'fontsize',2);
ylabel("y",'fontsize',2);
zlabel("z",'fontsize',2);
h_axes = gca();
h_axes.font_size = 2;
h_axes.isoview = "on";
h_axes.box = "off";
//h_axes.rotation_angles = [63.5,-127];
h_axes.rotation_angles = [0,0];
h_axes.data_bounds = [-2,-0.5,-0.4;2,0.5,0.4];
xgrid;

//hay que transformar las coordenadas de la catenaria de x-y a x-z
z1=y1; //cambio de ejes la catenaria
z2=y2;

// Animation Loop
//figure(2)
//dibujo las catenarias
for i_time=2:n_time

    drawlater();
        //hago que las catenarias se muevan con los drones
        param3d(vectorX1+vector_auxiliar2,vector_auxiliar3,matriz_y1(i_time,:));
        d=gce();
        d.foreground=color('blue');
        //set(gca(),"auto_clear","on")
        param3d(vectorX2+vector_auxiliar2,vector_auxiliar3,matriz_y2(i_time,:));
        e=gce();
        e.foreground=color('red');
        
       // set(gca(),"auto_clear","on")
    
    //shape1
    h_fac3d.data.x = PatchData_X(:,:,i_time);
    h_fac3d.data.y = PatchData_Y(:,:,i_time);
    h_fac3d.data.z = PatchData_Z(:,:,i_time);
    
   
   //shape2
    h_fac3d1.data.x = PatchData_X1(:,:,i_time);
    h_fac3d1.data.y = PatchData_Y1(:,:,i_time);
    h_fac3d1.data.z = PatchData_Z1(:,:,i_time);
    
    
    //shape3
    h_fac3d2.data.x = PatchData_X2(:,:,i_time);
    h_fac3d2.data.y = PatchData_Y2(:,:,i_time);
    h_fac3d2.data.z = PatchData_Z2(:,:,i_time);
   

    drawnow();
     
     
end

endfunction

//x=rand(1,2);
//y=zeros(1,2);
//z=rand(1,2)
//param3d(x,y,z);
//endfunction



