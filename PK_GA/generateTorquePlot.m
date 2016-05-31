function [t, tau] = generateTorquePlot(cfgInit, splineCoeff)
	% Given the coefficients for the free variable cublic splines, produce a plot of joint torques.
	%   Outputs:
	%
	%		t 				N x 1 Vector of normalized time steps.
	%						The program assumes the entire sequence takes 1 second to execute,
	%						with a 0.01 time step.
	%						N is equal to 100.
	%		
	%		tau				N x 4 Vector of torques. Each row is the torque vector for that
	%						time step. tau(i, :) = 
	%						[ankle torque, knee torque, hip torque, shoulder torque]
	%
	%	Inputs:
	%
	%		cfgInit			1 x 3 Vector of initial angles for the free variables.
	%						cfgInit = [alpha_init, theta_3_init, theta_4_init]
	%
	%		splineCoeff 	1 x 9 Vector of spline coefficients for the free variables
	%						c = splineCoeff
	%						alpha(t) 	= c(1)t^3 + c(2)t^2 + c(3)*t + cfgInit(1)
	%						theta_3(t) 	= c(4)t^3 + c(5)t^2 + c(6)*t + cfgInit(2)
	%						theta_4(t) 	= c(7)t^3 + c(8)t^2 + c(9)*t + cfgInit(3)
	%						These coefficients are generated from:
	%						optimize_crawl.m
	%						
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com
	%


	%%% Create Torque Function %%%
	% Path to joint torque table.
	[foldername, ~, ~] = fileparts(mfilename('fullpath'));
	cd(foldername)
	cd('..')
	addpath(fullfile(pwd, 'Data'));

	% Grab the torque data.
	tq = load(fullfile(pwd, 'Data', 'vrep3_TorqueTable1_5deg.txt'));

	% Torque table interpolation variables.
	X1 = tq(:,6); 		% alpha
	X2 = tq(:,3); 		% theta3
	X3 = tq(:,4); 		% theta4
	T1 = tq(:,11); 		% ankle pitch torque
	T2 = tq(:,12); 		% knee pitch torque
	T3 = tq(:,13); 		% hip pitch torque
	T4 = tq(:,14); 		% shoulder pitch torque

	% Create torque interpolants.
	% Use both nearest neighbor and linear interpolants (with linear preferred unless NaN).
	T1_interp = TriScatteredInterp(X1, X2, X3, T1, 'linear');
	T2_interp = TriScatteredInterp(X1, X2, X3, T2, 'linear');
	T3_interp = TriScatteredInterp(X1, X2, X3, T3, 'linear');
	T4_interp = TriScatteredInterp(X1, X2, X3, T4, 'linear');
	T1_interp_n = TriScatteredInterp(X1, X2, X3, T1, 'nearest');
	T2_interp_n = TriScatteredInterp(X1, X2, X3, T2, 'nearest');
	T3_interp_n = TriScatteredInterp(X1, X2, X3, T3, 'nearest');
	T4_interp_n = TriScatteredInterp(X1, X2, X3, T4, 'nearest');

	%%% Generate time sequence %%%
	% Final time (normalized).
	t_final = 1;

	% Sequence of times between 0 and t_final -- used to generate time sequence of angles.
	t_step = 0.01;
	t = (0:t_step:t_final)';

	% Calculate time sequence of angle values for (\alpha, \theta_3, \theta_4)
  	% corresponding to cubic spline coefficients given in splineCoeff.
  	c = splineCoeff;
  	alpha_t 	= c(1)*t.^3 + c(2)*t.^2 + c(3)*t + cfgInit(1);
  	theta_3_t 	= c(4)*t.^3 + c(5)*t.^2 + c(6)*t + cfgInit(2);
	theta_4_t 	= c(7)*t.^3 + c(8)*t.^2 + c(9)*t + cfgInit(3);

	% Grab torque values at the computed angle values.
	% Use both nearest neighbor and linear interpolants.
	% Linear Interpolation.
	T1_ 	= T1_interp(alpha_t, theta_3_t, theta_4_t);
	T2_ 	= T2_interp(alpha_t, theta_3_t, theta_4_t);
	T3_ 	= T3_interp(alpha_t, theta_3_t, theta_4_t);
	T4_ 	= T4_interp(alpha_t, theta_3_t, theta_4_t);
	% Nearest Neighbor Interpolation.
	T1_n 	= T1_interp_n(alpha_t, theta_3_t, theta_4_t);
	T2_n 	= T2_interp_n(alpha_t, theta_3_t, theta_4_t);
	T3_n 	= T3_interp_n(alpha_t, theta_3_t, theta_4_t);
	T4_n 	= T4_interp_n(alpha_t, theta_3_t, theta_4_t);
	
	% Prefer linear unless interpolated value is NaN (i.e., if angles outside convex hull 
	% of provided empirical data used for interpolation).
	T1_(isnan(T1_)) = T1_n(isnan(T1_));
	T2_(isnan(T2_)) = T2_n(isnan(T2_));
	T3_(isnan(T3_)) = T3_n(isnan(T3_));
	T4_(isnan(T4_)) = T4_n(isnan(T4_));

	% Plot the torques.
	figure(1);
	set(0,'defaultaxesfontsize',14);
	line_width=2;
	font_size=14;

	plot(t, T1_, t, T2_, t, T3_, t, T4_);
	xlabel('t (s)','FontSize',font_size);
	ylabel('\tau (s)','FontSize',font_size);
	title('Plot of joint torques for configuration');
	legend('Ankle', 'Knee', 'Hip', 'Shoulder');

	% Display the limits of the free variables (configuration angles).
	t_text = 'Range(\alpha): [';
	t_min = num2str(min(X1));
	t_max = num2str(max(X1));
	var_text = [t_text, t_min, ', ', t_max, ']'];
	disp(var_text);

	t_text = 'Range(\theta_3): [';
	t_min = num2str(min(X2));
	t_max = num2str(max(X2));
	var_text = [t_text, t_min, ', ', t_max, ']'];
	disp(var_text);

	t_text = 'Range(\theta_4): [';
	t_min = num2str(min(X3));
	t_max = num2str(max(X3));
	var_text = [t_text, t_min, ', ', t_max, ']'];
	disp(var_text);

	% Generate output vector.
	tau = [T1_, T2_, T3_, T4_];

	