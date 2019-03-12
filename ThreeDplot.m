%% Space Robotics and Autonomy - EEEM029
% Coursework 4/12/18
% Rachel Wiles 
% Student ID 6553707

% q4) plotting a graph to show 3D plot of final position of robot arm.
% x,y,z, variables found through PID controller. 

function ThreeDplot(x,y,z)

X = x.data;
Y = y.data;
Z = z.data;

plot3(X, Y, Z)
xlabel ('X axis')
ylabel ('Y axis')
zlabel ('Z axis')
title ('Movement of robot arm under PID controller')


end