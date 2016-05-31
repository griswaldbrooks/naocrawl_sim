function [t, q] = computeJointSequence(qi, qf, ti, tf, dt)
	% Take initial robot state, initial time, final time, and time increment and
	% compute the time sequence of states. 
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
	%		qi   	N x 1 vector of initial states
	%
	%		qf   	N x 1 vector of final states
	%
	%       ti      Time when simulation starts
	%
	%       tf      Time when simulation ends
	%
	%       dt   	Time increment between time instants
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	%%% Calculate time-state sequence %%%

	% Generate joint angles via constant theta3, theta4 and
	% linearly interpolating alpha
	[t,q] = computeJointSequenceLinearAlpha(ti, tf, dt);


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

function tau = g_returnTorqueFromTableVector(theta_cfg)
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
	%		theta_cfg	M x 3 vector of configuration parameters
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
	RAnklePitchTorque 		= griddata(ppCfgTable(:,1), ppCfgTable(:,2), ppCfgTable(:,3), torqueTable(:,1), theta_cfg(:,1),theta_cfg(:,2),theta_cfg(:,3),'nearest'); 
	RKneePitchTorque 		= griddata(ppCfgTable(:,1), ppCfgTable(:,2), ppCfgTable(:,3), torqueTable(:,2), theta_cfg(:,1),theta_cfg(:,2),theta_cfg(:,3),'nearest'); 
	RHipPitchTorque 		= griddata(ppCfgTable(:,1), ppCfgTable(:,2), ppCfgTable(:,3), torqueTable(:,3), theta_cfg(:,1),theta_cfg(:,2),theta_cfg(:,3),'nearest'); 
	RShoulderPitchTorque 	= griddata(ppCfgTable(:,1), ppCfgTable(:,2), ppCfgTable(:,3), torqueTable(:,4), theta_cfg(:,1),theta_cfg(:,2),theta_cfg(:,3),'nearest'); 

	% Return the interpolated torque vector
	tau = [RAnklePitchTorque, RKneePitchTorque, RHipPitchTorque, RShoulderPitchTorque];
%%% OTHER FUNCTIONS %%%

function tau = returnTorque(q)
	% Given a joint configuration, compute the vector of joint torques.
	% 
	%   Outputs:
	%		
	%		tau		N x 1 vector of joint torques.
	%
	%	Inputs:
	%
	%		q   	N x 1 vector of joint variables.
	%
	% Nao Statistics
	%
	% Motor Stall Torques (mNm)
	% Leg Motor: 68
	% Shoulder Motor: 14.3
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com


	% Gravity (m/sec^2)
	g = -9.8;

	% Get link lengths and masses
	[M, L] = returnNaoModelParameters();
	% Convert from millimeters to meters
	L = (1/1000)*L;

	% Using pseudo-static dynamics to calculate joint torques
	% tau = G(q) + J(q)'F

	% Compute sin cos vector of summed q's
	sinqn = [sin(q(1)), sin(q(1) + q(2)), sin(q(1) + q(2) + q(3)), sin(q(1) + q(2) + q(3) + q(4)), sin(q(1) + q(2) + q(3) + q(4) + q(5))]';
	cosqn = [cos(q(1)), cos(q(1) + q(2)), cos(q(1) + q(2) + q(3)), cos(q(1) + q(2) + q(3) + q(4)), cos(q(1) + q(2) + q(3) + q(4) + q(5))]';

	% Calculate the Gravity Vector
	G = g*(L.*M.*cosqn);

	% Calculate the Jacobian
	J = [
	 -L(1)*sinqn(1) - L(2)*sinqn(2) - L(3)*sinqn(3) - L(4)*sinqn(4) - (L(5)*sinqn(5))/2, -L(2)*sinqn(2) - L(3)*sinqn(3) - L(4)*sinqn(4) - (L(5)*sinqn(5))/2, -L(3)*sinqn(3) - L(4)*sinqn(4) - (L(5)*sinqn(5))/2, -L(4)*sinqn(4) - (L(5)*sinqn(5))/2, -(L(5)*sinqn(5))/2;
	  L(1)*cosqn(1) + L(2)*cosqn(2) + L(3)*cosqn(3) + L(4)*cosqn(4) + (L(5)*cosqn(5))/2,  L(2)*cosqn(2) + L(3)*cosqn(3) + L(4)*cosqn(4) + (L(5)*cosqn(5))/2,  L(3)*cosqn(3) + L(4)*cosqn(4) + (L(5)*cosqn(5))/2,  L(4)*cosqn(4) + (L(5)*cosqn(5))/2,  (L(5)*cosqn(5))/2;
	                                                                        		  0,  		                                                          0,  		                                          0, 	                              0,                  0;
	                                                                        		  0,  		                                                          0,  		                                          0, 	                              0,                  0;
	                                                                        		  0,  		                                                          0,  		                                          0, 	                              0,                  0;
	                                                                        		  1,  		                                                          1,  		                                          1, 	                              1,                  1];

	% Calculate the projected position of each mass (which is concentrated at the joints)
	x = cumsum(L(1:4).*cosqn(1:4));
	% Foregoing this point mass since it is on the ground
	x_elbow = L'*cosqn;

	% Total mass
	m_sum = sum(M(1:4));

	% Center of gravity in the X direction
	x_cog = (M(1:4)'*x)/m_sum;

	% Force on the elbow
	% Proportion of mass on elbow
	pElbow = abs(x_elbow - x_cog)/x_elbow;
	F_elbow = -g*m_sum*pElbow;
	F = [-F_elbow, F_elbow,0,0,0,0]';

	% Calculate the torques seen at the joints
	tau = G + J'*F;

function c = returnCost(q, alpha)
	% Compute a cost for being at a particular joint configuration.
	% 
	%   Outputs:
	%		
	%		c		Scalar value which is a cost for being at that state
	%
	%	Inputs:
	%
	%		q   	N x 1 vector of joint variables.
	%	
	%		alpha	Elbow (end effector) orientation wrt the floor. Typically
	%				a negative angle.
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
	qf = [ 54.95853, -79.70721, 16.5, 27.5, -109.2513]';
	qf = deg2rad(qf);
	alphaf = -90;
	alphaf = deg2rad(alphaf);


	% Get torques at this configuration
	tau = returnTorque(q);
	% Because the leg motors are more powerful than the arm motors
	% we prefer to use them so reduce their cost
	% Discount = Shoulder Motor Stall Torque / Leg Motor Stall Torque
	LEG_MOTOR_DISCOUNT = 14.3/68;
	motor_weighting = LEG_MOTOR_DISCOUNT*ones(1,length(tau));
	% Reset the Shoulder Motor weight
	motor_weighting(end) = 1;
	tau_cost = norm(motor_weighting*tau)^2;

	% Calculate the joint error cost
	q_err = norm(qf - q)^2 + norm(alphaf - alpha)^2;
	% Error gain
	k = 0.5;

	% Calculate the final state cost
	c =  (1 - k)*tau_cost+ k*q_err;

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
	theta_cfg = [q(3), q(4), alpha]';
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

function c = returnCostCfgVector(theta_cfg)
	% Compute a cost for being at a particular joint configuration.
	% 
	%   Outputs:
	%		
	%		c		Scalar value which is a cost for being at that state
	%
	%	Inputs:
	%
	%		theta_cfg	M x 3 vector of configuration parameters
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
	theta_cfg_FINAL = deg2rad([ 16.5, 27.5, -90]);
	theta_cfg_f = zeros(size(theta_cfg));
	for itr = 1:length(theta_cfg_FINAL)
		theta_cfg_f(:,itr) = theta_cfg_f(:,itr) + theta_cfg_FINAL(itr);
	end

	% Get torques at this configuration
	tau = g_returnTorqueFromTableVector(theta_cfg);
	% Because the leg motors are more powerful than the arm motors
	% we prefer to use them so reduce their cost
	% Discount = Shoulder Motor Stall Torque / Leg Motor Stall Torque
	LEG_MOTOR_DISCOUNT = 14.3/68;
	motor_weighting = LEG_MOTOR_DISCOUNT*ones(1,size(tau,2));
	% Reset the Shoulder Motor weight
	motor_weighting(end) = 1;
	% Update tau cost
	tau_cost = zeros(size(tau));
	for itr = 1:length(motor_weighting)
		tau_cost(:,itr) = motor_weighting(itr)*tau(:,itr);
	end
	tau_cost = sum(tau_cost, 2).^2;

	% Calculate the joint error cost
	theta_cfg_err = sum(theta_cfg_f - theta_cfg, 2).^2;
	% Error gain
	k = 0.5;

	% Calculate the final state cost
	c =  (1 - k)*tau_cost + k*theta_cfg_err;

function [t,q] = computeJointSequenceLinearAlpha(ti, tf, dt)
	% Heuristic based sequence generator. Generates a joint sequence by holding
	% theta3 and theta4 constant and linearly interpolating through alpha
	% from -30 degrees to -90 degrees.
	%   Outputs:
	%		
	%		t 		M x 1 Vector of time elements for each state. Vector is 
	%				length (tf - ti)/dt
	%
	%		q 		M x N Vector of state elements. 
	%				M is equal to (tf - ti)/dt, N is the number of state variables
	%
	%	Inputs:
	%
	%       ti      Time when simulation starts
	%
	%       tf      Time when simulation ends
	%
	%       dt   	Time increment between time instants
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	%%% Calculate time-state sequence %%%

	% Compute time vector
	t = ti:dt:tf;

	% The elbow angle alpha ranges from -30 degrees to -90 degrees. 
	% Alternatively, it should be calculated from qi and qf because
	% alpha is equal to the sum of the joint angles.
	alpha_i = deg2rad(-30);
	alpha_f = deg2rad(-90);
	% Iteration increment is so that alpha_i corresponds to ti
	% and alpha_f corresponds to tf
	alpha_inc = (alpha_f - alpha_i)/length(t);

	% Set theta3 theta4 constant
	theta3 = deg2rad(16.5);
	theta4 = deg2rad(27.5);	

	% Initialize joint angle vector
	N = 5; % Number of joint variables [theta1, theta2, theta3, theta4, theta5]
	q = zeros(length(t),N);
	q_itr = 1;

	% Compute joint angles
	% Method of computing joint angles by linearly iterating through alpha
	for alpha = alpha_i:alpha_inc:alpha_f

		% Set configuration angles
		theta_cfg = [theta3, theta4, alpha]';

		% Get joint configuration
		q(q_itr,:) = returnConstrainedJointConfiguration(theta_cfg)';

		% Increment joint angle vector iterator
		q_itr = q_itr + 1;
	end

function [t,q] = computeJointSequenceDPTorqueTable(ti, tf, dt)
	% Produces an optimal joint sequence according to the minimization of torques.
	% Torques are produced from a table of recorded torques that are interpolated.
	% Sequence is optimized using dynamic programming.
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
	%       ti      Time when simulation starts
	%
	%       tf      Time when simulation ends
	%
	%       dt   	Time increment between time instants
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	%%% Calculate time-state sequence %%%

	% Compute time vector
	t = ti:dt:tf;

	% Include requisite libraries
	[foldername, ~, ~] = fileparts(mfilename('fullpath'));
	cd(foldername)
	cd('..')
	addpath(fullfile(pwd, 'npermutek'));

	% Initialize torque function
	nameOfTorqueTableFile = fullfile(pwd, 'Data','vrep3_TorqueTable1_2_5deg.txt');
	g_initTorqueFromTable(nameOfTorqueTableFile);	

	% The elbow angle alpha ranges from -30 degrees to -90 degrees. 
	% Alternatively, it should be calculated from qi and qf because
	% alpha is equal to the sum of the joint angles.
	alpha_i = deg2rad(-30);
	alpha_f = deg2rad(-90);
	% Iteration increment is so that alpha_i corresponds to ti
	% and alpha_f corresponds to tf
	alpha_inc = (alpha_f - alpha_i)/length(t);

	% Initialize joint angle vector
	q = zeros(length(t),length(qi));
	q_itr = 1;

	% Set theta3 theta4 constant
	theta3 = deg2rad(16.5);
	theta4 = deg2rad(27.5);	

	% Set initial and final configurations
	theta_cfg_i = [theta3, theta4, alpha_i]';
	theta_cfg_f = [theta3, theta4, alpha_f]';

	% Set limits on configuration states
	stateLimits = deg2rad([	 -5,  45;
								-30,  15;
								-90, -30]);

	% Generate action space
	A = npermutek(deg2rad([-5,0,5]), length(theta_cfg_i));

function [Nodes, Adj] = returnGraph(depth, actions, initialState, finalState, stateLimits)
	% Returns a graph to be searched.
	%   Outputs:
	%		
	%		Adj				Sparse adjacency matrix representing tree graph. The value
	%						at each entry represents the cost to get there.
	%
	%		Nodes 			N x M vector representing the states and their configuration
	%						values. For example, Nodes(5, :) = [theta3, theta4, alpha]
	%						would be the state value for the node 5 in the graph.
	%						N = (size(actions,1)^(depth + 1)) - 1
	%
	%	Inputs:
	%
	%       depth			Scalar value of how many layers the tree should have. 
	%						Depth of zero will return only the root node.
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
	%		initialState	Root node.
	%
	%		finalState		Goal state, used as a stopping condition if needed.
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

	% TODO: ALL OF THIS NEEDS TO BE DOUBLE CHECKED BEFORE TESTING

	% Size of action space
	A = size(actions,1);

	% Number of nodes
	N = floor((A^(depth + 1) - 1)/(A - 1));

	% Init node list
	Nodes = zeros(N, length(initialState));

	% Generate list of nodes and where they lead with their costs 
	% Each row has the format:
	% [source, destination, cost-to-get-there]

	%%% Initialize source, destination, and cost lists
	% List length (Number of edges) is one less than the number of nodes
	listLength = N - 1;
	
	% The edges are always one less than the number of nodes in a tree like this
	% (But when there are multiple paths to a node there will be more)
	weights	= zeros(listLength,1);
	
	% Because states are enumerated simply the following parent-child structure will
	% always be
	children = 2:N;

	numberOfParents = floor((A^(depth) - 1)/(A - 1));
	parents	= 1:numberOfParents;
	parents = repmat(parents,A,1);
	parents = reshape(parents, 1, listLength);

	% Initialize node
	Nodes(1,:) = initialState';

	% Populate lists
	for parentItr = 1:listLength

		% Child iterator 
		cItrFirst = A*(parentItr - 1) + 2;
		cItrLast  = A*(parentItr - 1) + A;

		% Add state to the node list
		cfgState = Nodes(parentItr,:);

		% Generate children
		[children, costs] = returnChildren(cfgState, actions, stateLimits);

		% Populate nodes and weights
		Nodes(cItrFirst:cItrLast, :) = children;
		weights((cItrFirst-1):(cItrLast-1)) = costs;


		% If state equals final state
		if cfgState == finalState'
		end
	end

	% Populate end of node list with bottom layer

	% Generate sparse matrix
	M = sparse(parents, children, weights, N, N);

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
	% TODO: Get the limits to work 3/5/2015
	% 	% if not((sum(childCfg < stateLimits(:,1)) > 0) | (sum(childCfg > stateLimits(:,2)) > 0))
	costs = returnCostCfgVector(children);