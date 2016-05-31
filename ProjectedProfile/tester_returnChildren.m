function tester_returnChildren()
	% Quick script to test the returnChildren function
	
	% Include requisite libraries
	[foldername, ~, ~] = fileparts(mfilename('fullpath'));
	cd(foldername)
	cd('..')
	addpath(fullfile(pwd, 'npermutek'));

	% Initialize torque function
	nameOfTorqueTableFile = fullfile(pwd, 'Data','vrep3_TorqueTable1_2_5deg.txt');
	g_initTorqueFromTable(nameOfTorqueTableFile);

	% Initial alpha
	alpha_i = deg2rad(-30);
	% Set theta3 theta4 constant
	theta3 = deg2rad(16.5);
	theta4 = deg2rad(27.5);	

	% Initial configuration
	theta_cfg_i = [theta3, theta4, alpha_i]';

	% Set limits on configuration states
	stateLimits = deg2rad([	 -5,  45;
								-30,  15;
								-90, -30]);
	
	% Set of actions
	actions = npermutek(deg2rad([-5,0,5]), length(theta_cfg_i));

	[children, costs] = returnChildren(theta_cfg_i, actions, stateLimits)

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
function c = returnCostCfg(theta_cfg)
	% Compute a cost for being at a particular joint configuration.
	% 
	%   Outputs:
	%		
	%		c		Scalar value which is a cost for being at that state
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


	% Final joint configuration in degrees (goal joint angles)
	theta_cfg_f = deg2rad([ 16.5, 27.5, -90]');

	% Get torques at this configuration
	tau = g_returnTorqueFromTable(theta_cfg);
	% Because the leg motors are more powerful than the arm motors
	% we prefer to use them so reduce their cost
	% Discount = Shoulder Motor Stall Torque / Leg Motor Stall Torque
	LEG_MOTOR_DISCOUNT = 14.3/68;
	motor_weighting = LEG_MOTOR_DISCOUNT*ones(1,length(tau));
	% Reset the Shoulder Motor weight
	motor_weighting(end) = 1;
	tau_cost = norm(motor_weighting*tau)^2;

	% Calculate the joint error cost
	theta_cfg_err = norm(theta_cfg_f - theta_cfg)^2;
	% Error gain
	k = 0.5;

	% Calculate the final state cost
	c =  (1 - k)*tau_cost+ k*theta_cfg_err;


function [children, costs] = returnChildren(theta_cfg, actions, stateLimits)
	% Given an initial state, actions, and state limits, returns a list of children
	% and the cost associated with getting there. 
	%   Outputs:
	%		
	%		children		N x M vector of next states from q. N = size(actions,1).
	%						M is length(q).
	%
	%		costs 			N x 1 vector of costs associated with getting to that node.
	%
	%	Inputs:
	%
	%       theta_cfg		M x 1 state vector that is the parent state.
	%
	%		actions			N x M vector of actions. N is the number of possible
	%						actions each node can perform. M is the length of the 
	%						state vector. Example:
	%						[	 0,  0,  0;
	%							-5,  0,  0;
	%							-5, -5,  0;
	%							-5, -5, -5];
	%						Would be a set of four actions the 3x1 state vector
	%						could perform. If every variable could be -5, 0, or 5
	%						then N = 
	%
	%		stateLimits		M x 2 vector which contains the min and max value of 
	%						each state element.
	%						Example: 
	%							If q = [theta3, theta4, alpha] and theta3 ranged
	%							from -5 to 45 
	%							stateLimits = 
	%							[	 -5,  45;
	%								-30,  15;
	%								-90, -30];
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	%%% Generate children %%%
	% Initialize children
	children = zeros(size(actions));
	% Add q to actions to get next states
	for itr = 1:length(theta_cfg)
		children(:, itr) = theta_cfg(itr) + actions(:,itr);
	end

	%%% Generate costs %%%
	% Initialize costs
	costs = inf(size(actions,1), 1);
	% Update costs
	for itr = 1:length(costs)
		% Grab child
		childCfg = children(itr, :)';

		% If the child is out of range, let it's cost be infinite
		% if not((sum(childCfg < stateLimits(:,1)) > 0) | (sum(childCfg > stateLimits(:,2)) > 0))
			costs(itr) = returnCostCfg(childCfg);
		% end
	end