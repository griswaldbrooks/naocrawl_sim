function q = returnConstrainedJointConfiguration(theta_cfg)
	% Given a set of free variables, return the complete set of joint angles 
	% considering the kinematic constraints
	% 
	%   Outputs:
	%		
	%		q   		N x 1 vector of joint variables.
	%
	%	Inputs:
	%
	%		theta_cfg   M x 1 vector of configurable joint variables in radians. 
	%					In the case of Nao crawling using the projected profile method,
	%					M = 3 and are theta_cfg = [theta_3, theta_4, alpha]'
	%
	% Nao Statistics
	%
	% Joint Variables [theta1, theta2, theta3, theta4, theta5]
	% Initial joint configuration in radians 	(alpha = -38 degrees)
	% [ 1.6476      -2.223     0.28798     0.47997    -0.85573 ]
	%
	% Sample 2 joint configuration in radians  	(alpha = -55 degrees)
	% [ 1.4809     -2.0064     0.28798     0.47997     -1.2023 ]
	%
	% Sample 3 joint configuration in radians  	(alpha = -72 degrees)
	% [ 1.2565     -1.7374     0.28798     0.47997     -1.5437 ]
	%
	% Final joint configuration in radians  	(alpha = -90 degrees)
	% [ 0.95921     -1.3912     0.28798     0.47997     -1.9068 ]
	%
	% alpha range [-38,-90]
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com


	[~, L] = returnNaoModelParameters();
	l1 = L(1);
	l2 = L(2);
	l3 = L(3);
	l4 = L(4);
	l5 = L(5);

	% xd is the distance in mm from where the toes touch the ground
	% to where the elbows touch the ground. This distance is fixed.
	xd = 430;

	% Set the configurable joint variables from theta_cfg
	theta3 = theta_cfg(1);
	theta4 = theta_cfg(2);
	alpha  = theta_cfg(3);	

	% Use the Inverse Kinematics of the constrained profile to 
	% compute the other joint angles
	x_hat = xd - l5*cos(alpha); 
	y_hat = -l5*sin(alpha);
	k1 = l2 + l3*cos(theta3) + l4*cos(theta3 + theta4);
	k2 = -l3*sin(theta3) - l4*sin(theta3 + theta4);
	k5 = (x_hat^2 + y_hat^2 - l1^2 - k1^2 - k2^2)/(2*l1);
	A = -k1 - k5;
	B = 2*k2;
	C = k1 - k5;
	tan_gamma = roots([A,B,C]);
	theta2 = 2*atan(tan_gamma(1));
	k3 = l1 + k1*cos(theta2) + k2*sin(theta2);
	k4 = k2*cos(theta2) - k1*sin(theta2);
	costheta1 = (k3*x_hat - k4*y_hat)/(k3^2 + k4^2);
	sintheta1 = (k4*x_hat + k3*y_hat)/(k3^2 + k4^2);
	theta1 = atan2(sintheta1,costheta1);
	theta5 = alpha - theta1 - theta2 - theta3 - theta4;

	% Set the output variables
	q = [theta1, theta2, theta3, theta4, theta5]';
