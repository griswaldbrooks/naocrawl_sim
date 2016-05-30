function tester_TorqueTableInterp()
	% Test the return joint torques function interpolated from a file.
	% The file is a table of joint angles and torques.
	%   Outputs:
	%		
	%		<none>	Function plots torques for a given joint configuration.
	%
	%	Inputs:
	%
	%		<none>	Function interacts with user through prompts.
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	% Iterate through all alpha from -30:-5:-90
	% Iterate through all theta3 from -5: 5: 45
	% Iterate through all theta4 from 15:-5:-30

	% Initialize torque function
	% nameOfTorqueTableFile = '../Data/vrep3_TorqueTable1_5deg.txt';
	nameOfTorqueTableFile = '../Data/vrep3_TorqueTable1_2_5deg.txt';
	g_initTorqueFromTable(nameOfTorqueTableFile);	

	disp('Initial configuration in degrees is 	[-5,  15, -30]');
	disp('Final configuration in degrees is 	[45, -30, -90]');

	while(true)
		% Prompt user for configuration parameters
		disp('Enter configuration in degrees. Press Enter to exit. ')
		theta_cfg = input('[theta3, theta4, alpha]: ');
		if isempty(theta_cfg)
			break;
		end

		% Get configuration
		% q = returnConstrainedJointConfiguration(deg2rad(theta_cfg));

		% Get joint torques
		tau = g_returnTorqueFromTable(deg2rad(theta_cfg))

		% Plot returned torque
		disp(['Torques: [',num2str(tau'),']']);
		figure(1)
		thetaIndex = [2,3,4,5];
		barwidth1 = 0.5;
		bar(thetaIndex, tau, barwidth1,	'FaceColor',[0,0.7,0.7],...
		                     			'EdgeColor',[0,0.7,0.7])

		% legend('2.5 degree') % add legend
	end

%%%%%%%%%%%%%%%%%%%%%%%
%%% LOCAL FUNCTIONS %%%
%%%%%%%%%%%%%%%%%%%%%%%


%%% G_FUNCTIONS %%%

function g_initTorqueFromTable(nameOfTorqueTableFile)
	% Loads values from torque table file into an interpolatable
	% lattice.
	% 
	%   Outputs:
	%		
	%		<none>	Sets globals for use by other functions. Requires
	%				each row of the file be formatted as:
	%				[   theta1, 	theta2,		theta3,		theta4,		theta5, 	alpha,...	
	%					RAnklePitchAngle,   RKneePitchAngle, 	RHipPitchAngle, 	RShoulderPitchAngle, ...
	%					RAnklePitchTorque,  RKneePitchTorque, 	RHipPitchTorque, 	RShoulderPitchTorque]
	%
	%	Inputs:
	%
	%		nameOfTorqueTableFile	Name of the file to load from. 
	%
	% Nao Statistics
	%
	% Motor Stall Torques (mNm) (Before gear boxes)
	% Leg Motor: 68
	% Shoulder Motor: 14.3
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	global torqueTable ppJointTable ppCfgTable

	% Read in the torque table
	torqueTableRAW = csvread(nameOfTorqueTableFile);

	% Remove the first row because there was an error with
	% the table creation (getting the torques from V-REP) 
	% that makes the first set of torques NaN
	torqueTableRAW(1,:) = [];

	%%% Break the table up %%%
	% Table of projected profile configuration parameters
	ppCfgTable = [torqueTableRAW(:,3:4), torqueTableRAW(:,6)];
	
	% Table of projected profile joints angles
	ppJointTable = torqueTableRAW(:,1:5);
	
	% Table of V-REP Nao joint angles
	% naoJointTable = torqueTableRAW(:,7:10);

	% Table of V-REP Nap joint torques
	torqueTable = torqueTableRAW(:,11:14);

function tau = g_returnTorqueFromTable(theta_cfg)
	% Given a joint configuration, compute the vector of joint torques.
	% 
	%   Outputs:
	%		
	%		tau			4 x 1 vector of joint torques, corresponding to:
	%					[	RAnklePitchTorque,  
	%						RKneePitchTorque, 	
	%						RHipPitchTorque, 	
	%						RShoulderPitchTorque]
	%
	%	Inputs:
	%
	%		theta_cfg	3 x 1 vector of configuration parameters
	%					corresponding to: 
	%					theta_cfg = [theta3, theta4, alpha]'
	%
	% Nao Statistics
	%
	% Motor Stall Torques (mNm)
	% Leg Motor: 68
	% Shoulder Motor: 14.3
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	global torqueTable ppJointTable ppCfgTable

	% Use linear interpolation to get the torques from the table
	RAnklePitchTorque 		= griddata(ppCfgTable(:,1), ppCfgTable(:,2), ppCfgTable(:,3), torqueTable(:,1), theta_cfg(1),theta_cfg(2),theta_cfg(3),'nearest'); 
	RKneePitchTorque 		= griddata(ppCfgTable(:,1), ppCfgTable(:,2), ppCfgTable(:,3), torqueTable(:,2), theta_cfg(1),theta_cfg(2),theta_cfg(3),'nearest'); 
	RHipPitchTorque 		= griddata(ppCfgTable(:,1), ppCfgTable(:,2), ppCfgTable(:,3), torqueTable(:,3), theta_cfg(1),theta_cfg(2),theta_cfg(3),'nearest'); 
	RShoulderPitchTorque 	= griddata(ppCfgTable(:,1), ppCfgTable(:,2), ppCfgTable(:,3), torqueTable(:,4), theta_cfg(1),theta_cfg(2),theta_cfg(3),'nearest'); 

	% Return the interpolated torque vector
	tau = [RAnklePitchTorque, RKneePitchTorque, RHipPitchTorque, RShoulderPitchTorque]';

%%% OTHER FUNCTIONS %%%
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