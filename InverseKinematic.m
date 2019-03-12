%% Space Robotics and Autonomy - EEEM029
% Coursework 4/12/18
% Rachel Wiles 
% Student ID 6553707

function [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6] = InverseKinematic(T)

% Extracting variables from matrix
n_x = T(1,1);
n_y = T(2,1);
n_z = T(3,1);

s_x = T(1,2);
s_y = T(2,2);
s_z = T(3,2);

a_x = T(1,3);
a_y = T(2,3);
a_z = T(3,3);

p_x = T(1,4);
p_y = T(2,4);
p_z = T(3,4);


% From inspection of table values...
% Twist angles - degrees
alpha_1 = -90;
alpha_2 = 0;
alpha_3 = 90;
alpha_4 = -90;
alpha_5 = 90;
alpha_6 = 0;

% Link length - meters 
a_1 = 0;
a_2 = 0.5;
a_3 = 0;
a_4 = 0;
a_5 = 0;
a_6 = 0;

% Offset distance - meters
d_1 = 0;
d_2 = 0.25;
d_3 = 0;
d_4 = 1;
d_5 = 0;
d_6 = 0.5;


%% Calculate position coordinates of arm

% Lecture 3 Slide 15
position_arm = [p_x; p_y; p_z] - (d_6.*[a_x; a_y; a_z]);
parm_x = position_arm (1,1);
parm_y = position_arm (2,1);
parm_z = position_arm (3,1);


%% Calculate joint angle theta 1

% Lecture 3 Slide 18
theta_1 = (atan2d(parm_y, parm_x)) - (atan2d(d_2,(sqrt((parm_x^2)+(parm_y^2)-(d_2^2)))));


%% Calculate joint angle theta 2

% Lecture 3 Slide 18
c_1 = cosd(theta_1);
s_1 = sind(theta_1);

A = (c_1*parm_x) + (s_1*parm_y);
B = (A^2) + (parm_z^2) + (a_2^2) - (d_4^2);

theta_2 = (atan2d(A,parm_z)) - (atan2d(B, (sqrt((A^2) + (parm_z^2) - (B^2)))));


%% Calculate joint angle theta 3

% Lecture 3 slide 19
c_2 = cosd(theta_2);
s_2 = sind(theta_2);

theta_3 = (atan2d((A - (a_2*c_2)),(parm_z + (a_2*s_2)))) - theta_2;

 
%% Calculate joint angle theta 4

% Lecture 3 slide 20
c_23 = cosd(theta_2 + theta_3);
s_23 = sind(theta_2 + theta_3);

theta_4 = atan2d(((-s_1*a_x) + (c_1*a_y)),(c_23*((c_1*a_x) + (s_1*a_y)) - (s_23*a_z)));


%% Calculate joint angle theta 5

% Lecture 3 slide 20
theta_5 = atan2d((sqrt(((c_1*c_23*a_x) + (s_1*c_23*a_y) - (s_23*a_z))^2 + ((-s_1*a_x) + (c_1*a_y))^2)),(s_23*((c_1*a_x) + (s_1*a_y)) + (c_23*a_z)));


%% Calculate joing angle theta 6

% Lecture 3 slide 20
theta_6 = atan2d((s_23*((c_1*s_x) + (s_1*s_y)) + (c_23*s_z)), -(s_23*((c_1*n_x) + (s_1*n_y)) + (c_23*n_z)));


