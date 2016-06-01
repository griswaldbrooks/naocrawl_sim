%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% naoCrawlSimulation.m
%
% This script shows an example of how to use the MATLAB libraries
% written for getting the Nao to crawl using the projected profile
% method. It allows for the projected profile crawling gait to be 
% run as a MATLAB plot and a V-REP simulation
%
% Griswald Brooks
% griswald.brooks@gmail.com 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Reset plotting and variable spaces
cla, clc, clear all
% Set global plotting rules
axis equal
hold all

%%% Include Robot Libraries %%%
[foldername, ~, ~] = fileparts(mfilename('fullpath'));
% Library for plotting robots in MATLAB
addpath(genpath(fullfile(foldername, 'RobotPlotting')));
% Library to use V-REP with MATLAB
addpath(genpath(fullfile(foldername, 'V-RepApi')));
% Projected Profile Library
addpath(genpath(fullfile(foldername, 'ProjectedProfile')));
% PK Genetic Optimization
addpath(genpath(fullfile(foldername, 'PK_GA')));
% Temp
addpath(genpath(fullfile(foldername, 'temp')));



% Projected Profile Nao joint variables in radians
% q = [theta1, theta2, theta3, theta4, theta5]
qi = [ 1.6476,  -2.223,  0.28798, 0.47997, -0.85573 ]';
qf = [ 0.95921, -1.3912, 0.28798, 0.47997, -1.9068 ]';

% Tuned the hip joint a bit from the previous estimate
qi(4) = deg2rad(15);
qf(4) = deg2rad(15); 

% Time variables (seconds)
dt = 0.01;  % Time increment
tf = 1;
ti = 0;

disp('Computing joint trajectory.');

% Compute joint trajectory
[t, q] = computeJointSequence(qi, qf, ti, tf, dt);
% [t, q] = generatePKGASequence(qi, qf, ti, tf, dt);

% Save torque values to csv file
csvwrite(fullfile(foldername, 'temp', 'pp_Anglesm.txt'), [t, q]);	

input('Simulation computed. Press "Enter" to animate.');

% Show animation of gait
USER_ITERATED = false;
generateAnimation(q, t, USER_ITERATED);

%%% Send joint sequence to V-REP %%%

% Prompt user
input('Start Nao V-REP simulator. Press "Enter" to continue.');

% Connect to simulator
disp('Connecting to simulator.');
[clientID, vrep] = connectToVREP();

if (clientID > -1)
  disp('Connected to simulator.');
  % Send sequence
  disp('Sending joint sequence.');

  startNaoCrawl(clientID, vrep, q, t);

  % Disconnect from simulator
  disconnectFromVREP(clientID, vrep);
  disp('Simulation terminated.');

else  
  disp('Could not connect to simulator.');
end

input('Simulation complete. Press enter to plot additional graphs.');

% Show error and data plots
generatePlots();
