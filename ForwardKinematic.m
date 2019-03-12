function [x,y,z] = ForwardKinematic(theta_1,theta_2,theta_3,theta_4,theta_5,theta_6)

%% Space Robotics and Autonomy - EEEM029
% Coursework 4/12/18
% Rachel Wiles 
% Student ID 6553707


%% Setup
% Before running, run InverseKinematics.m to get the output value of theta to use in this code. 

newc_1 = cosd(theta_1);
newc_2 = cosd(theta_2);
newc_3 = cosd(theta_3);
newc_4 = cosd(theta_4);
newc_5 = cosd(theta_5);
newc_6 = cosd(theta_6);

news_1 = sind(theta_1);
news_2 = sind(theta_2);
news_3 = sind(theta_3);
news_4 = sind(theta_4);
news_5 = sind(theta_5);
news_6 = sind(theta_6);

% From inspection of table values...
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

%% Determine DH for the first 3 links

% Lecture 2 slide 33
A_01 = [newc_1 0 -news_1 0; news_1 0 newc_1 0; 0 -1 0 0; 0 0 0 1];

A_12 = [newc_2 -news_2 0 (a_2*newc_2); news_2 newc_2 0 (a_2*news_2); 0 0 1 d_2; 0 0 0 1];

A_23 = [newc_3 0 news_3 0; news_3 0 -newc_3 0; 0 1 0 0; 0 0 0 1];


% Combine
A_03 = A_01*A_12*A_23;

%% Determine DH for the last 3 links

% Lecture 2 slide 33

A_34 = [newc_4 0 -news_4 0; news_4 0 newc_4 0; 0 -1 0 d_4; 0 0 0 1];

A_45 = [newc_5 0 news_5 0; news_5 0 -newc_5 0; 0 1 0 0; 0 0 0 1];

A_56 = [newc_6 -news_6 0 0; news_6 newc_6 0 0; 0 0 1 d_6; 0 0 0 1];


%Combine
A_36 = A_34*A_45*A_56;


%% Combine these to form the origial DH used in the inverse

% Lecture 2 slide 34
new_T = A_03*A_36;


x = new_T(1,4);
y = new_T(2,4);
z = new_T(3,4);







