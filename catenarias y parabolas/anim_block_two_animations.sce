clear; xdel(winsid());

// Create data
t = 0:0.005:1;    // Time data
x1 = sin(2*%pi*t); // Position data 1
x2 = cos(2*%pi*t); // Position data 2

// Draw initial figure
figure(1);
drawlater();
plot(x1(1),0,'o');
h1 = gce();
h1.children.mark_size = 20;
h1.children.mark_background = 2;
plot(x2(1),0,'o');
h2 = gce();
h2.children.mark_size = 20;
h2.children.mark_background = 3;
h_axes = gca();
h_axes.data_bounds = [-1.5,-1.5;1.5,1.5];
drawnow();

// Animation Loop
i = 1;
while i<=length(t)
    drawlater();
    h1.children.data = [x1(i),0];
    h2.children.data = [x2(i),0];
    drawnow();
    i = i+1;
end
