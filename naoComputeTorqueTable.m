%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% naoComputeTorqueTable.m
%
% This script uses the V-REP simulator to build a table of joint
% angles and joint torques for the Projected Profile crawling
% algorithm.
%
% Griswald Brooks
% griswald.brooks@gmail.com	
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Include Robot Libraries %%%
[foldername, ~, ~] = fileparts(mfilename('fullpath'));
% Library to use V-REP with MATLAB
addpath(genpath(fullfile(foldername, 'V-RepApi')));
% Projected Profile Library
addpath(genpath(fullfile(foldername, 'ProjectedProfile')));


% Reset plotting and variable spaces
cla, clc, clear all
axis equal
hold all

% Projected Profile Nao joint variables in radians
% q = [theta1, theta2, theta3, theta4, theta5]
qi = [ 1.6476,  -2.223,  0.28798, 0.47997, -0.85573 ]';

% Tuned the hip joint a bit from the previous estimate
% qi(4) = deg2rad(15);

%%% Start V-REP %%%

% Prompt user
disp('Warning: This program will send many commands to the V-REP simulator and will take a long time to compute. ')
resp = input('Do you want to continue? (y/N)', 's');
if isempty(resp)
    disp('Terminating program.');
elseif (resp == 'y') || (resp == 'Y')
    input('Start Nao V-REP simulator. Press "Enter" to continue.');

    % Connect to simulator
    disp('Connecting to simulator.');
    [clientID, vrep] = connectToVREP();

    if (clientID > -1)
        disp('Connected to simulator.');
        % Send sequence
        disp('Starting joint torque table generator.');
        generateJointTorqueTable(clientID, vrep);

        % Disconnect from simulator
        disconnectFromVREP(clientID, vrep);
        disp('Simulation terminated.');

    else	
        disp('Could not connect to simulator.');
    end
end
