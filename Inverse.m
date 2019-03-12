%% Space Robotics and Autonomy - EEEM029
% Coursework 4/12/18
% Rachel Wiles 
% Student ID 6553707

% q1) Finding inverse kinematic solution for all possible arm
% configurations so as to reach the target position and orientation.

% 6dof, with the last joint being a spherical wrist, therefore the last 3
% dof intersect. Arm is joints 1-3, wrist is joints 4-6.

% Position of end effector is determined by configuration of arm.
% Orientation of end effector is determined by the rotation of the wrist. 

%% Setup - from question paper

% DH matrix
T = [-1/sqrt(2) 0 1/sqrt(2) 1; 0 -1 0 1; 1/sqrt(2) 0 1/sqrt(2) 0; 0 0 0 1];

% From inspection of DH matrix...
% Lecture 2 slide 33
n_x = -1/sqrt(2);
n_y = 0;
n_z = 1/sqrt(2);

s_x = 0;
s_y = -1;
s_z = 0;

a_x = 1/sqrt(2);
a_y = 0;
a_z = 1/sqrt(2);

p_x = 1;
p_y = 1;
p_z = 0;


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


%% Calculate joint angle theta 1 & display

% Lecture 3 Slide 18
theta_1_1 = (atan2d(parm_y, parm_x)) - (atan2d(d_2,(sqrt((parm_x^2)+(parm_y^2)-(d_2^2)))));


%% Calculate joint angle theta 2 & display

% Lecture 3 Slide 18
c_1 = cosd(theta_1_1);
s_1 = sind(theta_1_1);

A = (c_1*parm_x) + (s_1*parm_y);
B = (A^2) + (parm_z^2) + (a_2^2) - (d_4^2);

theta_2_1 = (atan2d(A,parm_z)) - (atan2d(B, (sqrt((A^2) + (parm_z^2) - (B^2)))));


%% Calculate joint angle theta 3 & display

% Lecture 3 slide 19
c_2 = cosd(theta_2_1);
s_2 = sind(theta_2_1);

theta_3_1 = (atan2d((A - (a_2*c_2)),(parm_z + (a_2*s_2)))) - theta_2_1;

 
%% Calculate joint angle theta 4 & display

% Lecture 3 slide 20
c_23 = cosd(theta_2_1 + theta_3_1);
s_23 = sind(theta_2_1 + theta_3_1);

theta_4_1 = atan2d(((-s_1*a_x) + (c_1*a_y)),(c_23*((c_1*a_x) + (s_1*a_y)) - (s_23*a_z)));


%% Calculate joint angle theta 5 & display

% Lecture 3 slide 20
theta_5_1 = atan2d((sqrt(((c_1*c_23*a_x) + (s_1*c_23*a_y) - (s_23*a_z))^2 + ((-s_1*a_x) + (c_1*a_y))^2)),(s_23*((c_1*a_x) + (s_1*a_y)) + (c_23*a_z)));


%% Calculate joing angle theta 6 & display

% Lecture 3 slide 20
theta_6_1 = atan2d((s_23*((c_1*s_x) + (s_1*s_y)) + (c_23*s_z)), -(s_23*((c_1*n_x) + (s_1*n_y)) + (c_23*n_z)));




%% Repeat with sign change 

theta_1_2 = (atan2d(parm_y, parm_x)) - (atan2d(d_2,(-sqrt((parm_x^2)+(parm_y^2)-(d_2^2))))); %Negative SQRT here

c_1 = cosd(theta_1_2);
s_1 = sind(theta_1_2);

A = (c_1*parm_x) + (s_1*parm_y);
B = (A^2) + (parm_z^2) + (a_2^2) - (d_4^2);

theta_2_2 = (atan2d(A,parm_z)) - (atan2d(B, (sqrt((A^2) + (parm_z^2) - (B^2)))));

c_2 = cosd(theta_2_2);
s_2 = sind(theta_2_2);

theta_3_2 = (atan2d((A - (a_2*c_2)),(parm_z + (a_2*s_2)))) - theta_2_2;

c_23 = cosd(theta_2_2 + theta_3_2);
s_23 = sind(theta_2_2 + theta_3_2);

theta_4_2 = atan2d(((-s_1*a_x) + (c_1*a_y)),(c_23*((c_1*a_x) + (s_1*a_y)) - (s_23*a_z)));

theta_5_2 = atan2d((sqrt(((c_1*c_23*a_x) + (s_1*c_23*a_y) - (s_23*a_z))^2 + ((-s_1*a_x) + (c_1*a_y))^2)),(s_23*((c_1*a_x) + (s_1*a_y)) + (c_23*a_z)));

theta_6_2 = atan2d((s_23*((c_1*s_x) + (s_1*s_y)) + (c_23*s_z)), -(s_23*((c_1*n_x) + (s_1*n_y)) + (c_23*n_z)));


%% Repeat with sign change
theta_1_3 = (atan2d(parm_y, parm_x)) - (atan2d(d_2,(-sqrt((parm_x^2)+(parm_y^2)-(d_2^2))))); %Negative SQRT here

c_1 = cosd(theta_1_3);
s_1 = sind(theta_1_3);

A = (c_1*parm_x) + (s_1*parm_y);
B = (A^2) + (parm_z^2) + (a_2^2) - (d_4^2);

theta_2_3 = (atan2d(A,parm_z)) - (atan2d(B, (-sqrt((A^2) + (parm_z^2) - (B^2))))); %Negative SQRT here

c_2 = cosd(theta_2_3);
s_2 = sind(theta_2_3);

theta_3_3 = (atan2d((A - (a_2*c_2)),(parm_z + (a_2*s_2)))) - theta_2_3;

c_23 = cosd(theta_2_3 + theta_3_3);
s_23 = sind(theta_2_3 + theta_3_3);

theta_4_3 = atan2d(((-s_1*a_x) + (c_1*a_y)),(c_23*((c_1*a_x) + (s_1*a_y)) - (s_23*a_z)));

theta_5_3 = atan2d((sqrt(((c_1*c_23*a_x) + (s_1*c_23*a_y) - (s_23*a_z))^2 + ((-s_1*a_x) + (c_1*a_y))^2)),(s_23*((c_1*a_x) + (s_1*a_y)) + (c_23*a_z)));

theta_6_3 = atan2d((s_23*((c_1*s_x) + (s_1*s_y)) + (c_23*s_z)), -(s_23*((c_1*n_x) + (s_1*n_y)) + (c_23*n_z)));


%% Repeat with sign change
theta_1_4 = (atan2d(parm_y, parm_x)) - (atan2d(d_2,(sqrt((parm_x^2)+(parm_y^2)-(d_2^2))))); 

c_1 = cosd(theta_1_4);
s_1 = sind(theta_1_4);

A = (c_1*parm_x) + (s_1*parm_y);
B = (A^2) + (parm_z^2) + (a_2^2) - (d_4^2);

theta_2_4 = (atan2d(A,parm_z)) - (atan2d(B, (-sqrt((A^2) + (parm_z^2) - (B^2))))); %Negative SQRT here

c_2 = cosd(theta_2_4);
s_2 = sind(theta_2_4);

theta_3_4 = (atan2d((A - (a_2*c_2)),(parm_z + (a_2*s_2)))) - theta_2_4;

c_23 = cosd(theta_2_4 + theta_3_4);
s_23 = sind(theta_2_4 + theta_3_4);

theta_4_4 = atan2d(((-s_1*a_x) + (c_1*a_y)),(c_23*((c_1*a_x) + (s_1*a_y)) - (s_23*a_z)));

theta_5_4 = atan2d((sqrt(((c_1*c_23*a_x) + (s_1*c_23*a_y) - (s_23*a_z))^2 + ((-s_1*a_x) + (c_1*a_y))^2)),(s_23*((c_1*a_x) + (s_1*a_y)) + (c_23*a_z)));

theta_6_4 = atan2d((s_23*((c_1*s_x) + (s_1*s_y)) + (c_23*s_z)), -(s_23*((c_1*n_x) + (s_1*n_y)) + (c_23*n_z)));



%% Combine thetas

theta_1 = [theta_1_1 theta_1_2 theta_1_3 theta_1_4]
theta_2 = [theta_2_1 theta_2_2 theta_2_3 theta_2_4]
theta_3 = [theta_3_1 theta_3_2 theta_3_3 theta_3_4]
theta_4 = [theta_4_1 theta_4_2 theta_4_3 theta_4_4]
theta_5 = [theta_5_1 theta_5_2 theta_5_3 theta_5_4]
theta_6 = [theta_6_1 theta_6_2 theta_6_3 theta_6_4]

