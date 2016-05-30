function [t, q] = generatePKGASequence(theta_cfg_i, theta_cfg_f, ti, tf, dt)
	% Take initial and final robot configuration, initial and final time, and 
	% time increment and compute the time sequence of states from PK's best
	% iteration of a MATLAB's genetic algorithm
	%
	%   Outputs:
	%		
	%		t 		M x 1 Vector of time elements for each state. Vector is 
	%				length (tf - ti)/dt
	%
	%		q 		M x N Vector of state elements. 
	%				N is equal to (tf - ti)/dt, M is the number of state variables
	%
	%	Inputs:
	%
	%		theta_cfg_i   	N x 1 vector of initial joint configurations
	%
	%		theta_cfg_f   	N x 1 vector of final joint configurations
	%
	%       ti      		Time when simulation starts
	%
	%       tf      		Time when simulation ends
	%
	%       dt   			Time increment between time instants
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	% Projected Profile
	addpath(fullfile(pwd,'..','Code','ProjectedProfile'));

	%%% Right now everything is constant until it can be generatized %%%
	t_step = 0.01;
	t_final = 1;

	% Generate time vector
	t = (0:t_step:t_final)';

	% specified initial and final values of (\alpha, \theta_3, \theta_4)
	x_init = [-30*pi/180 ; 0.28798  ;   0.47997];
	% x_final = [-90*pi/180 ; 0.28798  ;   0.47997];

	% best x value found after a few runs of the genetic algorithm
	% x_best = [ -1.9362   -0.8890    1.7781   -0.0872   -0.0099    0.0972    0.3388   -0.6390    0.3002];
	x_best = [-0.2134    1.1570   -1.9898    0.2365    0.0893   -0.3267    1.8796   -0.1365   -1.7434];

	% optimized trajectory
	x1_ = x_best(1)*t.^3 + x_best(2)*t.^2 + x_best(3)*t + x_init(1);
	x2_ = x_best(4)*t.^3 + x_best(5)*t.^2 + x_best(6)*t + x_init(2);
	x3_ = x_best(7)*t.^3 + x_best(8)*t.^2 + x_best(9)*t + x_init(3);


	% Initialize joint angle vector
	N = 5; % Number of joint variables [theta1, theta2, theta3, theta4, theta5]
	q = zeros(length(t),N);

	% Compute joint angles
	% Method of computing joint angles by linearly iterating through alpha
	for q_itr = 1:size(q,1)

		% Set configuration angles
		alpha  = x1_(q_itr);
		theta3 = x2_(q_itr);
		theta4 = x3_(q_itr);
		theta_cfg = [theta3, theta4, alpha]';

		% Get joint configuration
		q(q_itr,:) = returnConstrainedJointConfiguration(theta_cfg)';

		% Increment joint angle vector iterator
		q_itr = q_itr + 1;
	end

%%%%%%%%%%%%%%%%%%%%%%%
%%% LOCAL FUNCTIONS %%%
%%%%%%%%%%%%%%%%%%%%%%%

