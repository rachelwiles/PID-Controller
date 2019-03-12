%% Space Robotics and Autonomy - EEEM029
% Coursework 4/12/18
% Rachel Wiles 
% Student ID 6553707

% q2) Verifying results of inverse kinematic solution in Inverse.m by
% performing forward kinematics. 

%% Setup
% Before running, run Inverse.m to get the output value of theta to use in 
% this code. 
% This code will verify for the first set of thetas only. It can be
% repeated to run on the other 3 sets of theta angles if the following
% section of the code is altered to match the naming in Inverse.m

newc_1 = cosd(theta_1_1);
newc_2 = cosd(theta_2_1);
newc_3 = cosd(theta_3_1);
newc_4 = cosd(theta_4_1);
newc_5 = cosd(theta_5_1);
newc_6 = cosd(theta_6_1);

news_1 = sind(theta_1_1);
news_2 = sind(theta_2_1);
news_3 = sind(theta_3_1);
news_4 = sind(theta_4_1);
news_5 = sind(theta_5_1);
news_6 = sind(theta_6_1);


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
T
new_T = A_03*A_36




