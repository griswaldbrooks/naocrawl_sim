function startNaoCrawl(clientID, vrep, jointSequence, t)
	% Starts V-REP Nao crawling and reads motor torques.
	% 
	%   Outputs:
	%		
	%		<none>			Function interacts with V-REP.
	%
	%	Inputs:
	%
	%		clientID   		Nao's V-REP client ID returned from	vrep.simxStart.
	%
	%		vrep 			Object for calling V-REP simulator functions. 
	%						Returned from the remApi function.
	%
	%		jointSequence	M x N Vector of projected profile joint angles. 
	%						N is equal to (tf - ti)/dt, M is the number of state
	%						variables, which should be five.
	%
	%		t 				M x 1 Vector of time elements for each state. Vector is 
	%						length (tf - ti)/dt.
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com
	
	% Get path for saving torques.
	[foldername, ~, ~] = fileparts(mfilename('fullpath'));
	cd(foldername);
	cd('..');
	foldername = pwd;

	% Initialize global parameters
	if(g_initGlobalParameters(clientID, vrep))

		% Get all of the joint angles
		% jointMap = getJointAngles();

		% Matrix for recording torques
		recordedTorques = zeros(length(t), 4);

		% Iterate through all of the joints
    	for k = 1:length(t)
        	% Start the timer on the plotting function
        	tic

        	% Alpha is the sum of all joint angles
        	alpha = sum(jointSequence(k,:));
			
			% Compute new joint angles
			jointMap = crawl_ppArrayToJointMap(jointSequence(k,:), alpha);

			% Set V-REP Nao joints
			jointAnglesSentSuccessfully = setJointAngles(jointMap);
			
			if(~jointAnglesSentSuccessfully)
				disp(['Joint angles sent unsuccessfully. k = ', num2str(k)]);
			end

			% Find the dt of the time signal
            if k == length(t)
                dt = t(k) - t(k-1);
            else
                dt = t(k+1) - t(k);
            end
            
            % Grab the time it took to plot
            plottingTime = toc;

            % Adjust dt to compensate for plotting time
            if (dt > plottingTime)
                dt = dt - plottingTime;
            end

            % Pause animation to make it plot close to the actual
            % moving speed
            pause(dt);		

            % Read joint torques
			torqueMap = getJointTorques();

			% Record torque values
			recordedTorques(k,:) = [torqueMap('RAnklePitch'),torqueMap('RKneePitch'),torqueMap('RHipPitch'),torqueMap('RShoulderPitch')];
			% Print out torques			
			% pp_Torques = [torqueMap('RAnklePitch'),torqueMap('RKneePitch'),torqueMap('RHipPitch'),torqueMap('RShoulderPitch')];
            % input(['Joint Torques (Nm): [', num2str(pp_Torques),']']);

            % Display the iteration
            % disp(['k: ',num2str(k)]);
		end

		% Save torque values to csv file
		csvwrite(fullfile(foldername, 'temp',['vrepn_Torquesm_',num2str(t(end)),'.txt']), recordedTorques);

	end

%%%%%%%%%%%%%%%%%%%%%%%
%%% LOCAL FUNCTIONS %%%
%%%%%%%%%%%%%%%%%%%%%%%

%%% G_FUNCTIONS %%%
function success = g_initGlobalParameters(clientID, vrep)
	% Initializes all of the parameters and objects that are commonly
	% used by most of the functions. Required to be called before using
	% any functions prefixed with g_ such as g_returnVREPObjects.
	% 
	%   Outputs:
	%		
	%		success		Returns false if ANY of the initializations fail.
	%
	%	Inputs:
	%
	%		clientID   	Nao's V-REP client ID returned from	vrep.simxStart
	%
	%		vrep 		Object for calling V-REP simulator functions. 
	%					Returned from the remApi function.
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	global g_OPTION g_CLIENTID g_VREP g_HMAP

	% Set VREP parameters
	g_CLIENTID = clientID;
	g_VREP = vrep;

	% Global option for all functions
	% g_OPTION = vrep.simx_opmode_oneshot_wait;	
	g_OPTION = vrep.simx_opmode_streaming;

	% Create the joint handle map for referencing joints
	g_HMAP = g_getJointHandles(clientID, vrep);
    
    % Set output variable
    success = true;

function [clientID, vrep, option] = g_returnVREPobjects()
	% Returns commonly used V-REP objects.
	%
	%	Outputs:
	%
	%		clientID   	Nao's V-REP client ID returned from	vrep.simxStart
	%
	%		vrep 		Object for calling V-REP simulator functions. 
	%					Returned from the remApi function.
	%
	%		option 		Option used for getting and setting V-REP joints.
	%
	%	Inputs:
	%
	%		<none>   	Function is initialized via g_initGlobalParameters
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com	

	global g_CLIENTID g_VREP g_OPTION

	clientID = g_CLIENTID;
	vrep = g_VREP;
	option = g_OPTION;

function hMap = g_returnHandleMap()
	% Returns the V-REP handles for all of Nao's joints. Handles are in a map object
	% which is used to return the handle to be passed into the V-REP 
	% simxGetJointPosition and simxSetTargetJointPosition functions.
	% 
	%   Outputs:
	%		
	%		hMap		Map of Nao's joint handles. Given a key such as 
	%					'RKneePitch', it will return the appropriate joint
	%					handle.
	%
	%	Inputs:
	%
	%		<none>   	Function is initialized via g_initGlobalParameters
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com	

	global g_HMAP;
	hMap = g_HMAP;

function hMap = g_getJointHandles(clientID, vrep)
	% Gets the V-REP handles for all of Nao's joints. Puts them into map object
	% which is used to return the handle to be passed into the V-REP 
	% simxGetJointPosition and simxSetTargetJointPosition functions.
	% 
	%   Outputs:
	%		
	%		hMap		Map of Nao's joint handles. Given a key such as 
	%					'RKneePitch', it will return the appropriate joint
	%					handle.
	%
	%	Inputs:
	%
	%		clientID   	Nao's V-REP client ID returned from	vrep.simxStart
	%
	%		vrep 		Object for calling V-REP simulator functions. 
	%					Returned from the remApi function.
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	% Set the option for all functions
	option = vrep.simx_opmode_oneshot_wait;

	% Array of joint names
	vrepJointNames = returnVREPJointNames();

	% Array for joint handles
	jointHandles = zeros(1, length(vrepJointNames));

	% Get all of the joint handles
	for i = 1:length((vrepJointNames))
		[rtn, handle] = vrep.simxGetObjectHandle(clientID,vrepJointNames{i},option);
		if(rtn==vrep.simx_return_ok)
			jointHandles(i) = handle;
		else
			disp(['Failed to get handle for joint ', vrepJointNames{i}]);
		end
	end

	% Array of joint handle names
	jointNames = returnJointNames();

	% Produce the handle map
	hMap = containers.Map(jointNames, jointHandles);

%%% SETTERS AND GETTERS FUNCTIONS %%%

function vrepJointNames = returnVREPJointNames()
	% Returns a cell array of all the V-REP joint names for getting
	% joint handles.
	% 
	%   Outputs:
	%		
	%		vrepJointNames 	Cell array of joint name strings
	%
	%	Inputs:
	%
	%		<none>			List of joint names is fixed.
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	% Array of joint names
	vrepJointNames = {	'RHipYawPitch3#'
					    'RHipRoll3#'
					    'RHipPitch3#'
					    'RKneePitch3#'
					    'RAnklePitch3#'
					    'LHipYawPitch3#'
					    'LHipRoll3#'
					    'LHipPitch3#'
					    'LKneePitch3#'
					    'LAnklePitch3#'
					    'RShoulderPitch3#'
					    'RShoulderRoll3#'
					    'RElbowYaw3#'
					    'RElbowRoll3#'
					    'LShoulderPitch3#'
					    'LShoulderRoll3#'
					    'LElbowYaw3#'
					    'LElbowRoll3#'};

function jointAngle = readJointAngle(jointHandle)
	% Returns the joint angle for one joint in V-REP Nao.
	% 
	%   Outputs:
	%		
	%		jointAngle	Angle of joint in radians.
	%
	%	Inputs:
	%
	%		jointHandle	V-REP handle for the requested joint.
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	% Get V-REP objects
	[clientID, vrep, option] = g_returnVREPobjects();

	% Get the joint angle
	[rtn, position] = vrep.simxGetJointPosition(clientID,jointHandle,option);
	if(rtn==vrep.simx_return_ok)
		jointAngle = position;
	else
		jointAngle = NaN;
	end

function jointTorque = readJointTorque(jointHandle)
	% Returns the torque for one joint in V-REP Nao.
	% 
	%   Outputs:
	%		
	%		jointTorque		Angle of joint in radians.
	%
	%	Inputs:
	%
	%		jointHandle		V-REP handle for the requested joint.
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	% Get V-REP objects
	[clientID, vrep, option] = g_returnVREPobjects();

	% Get the joint angle
	[rtn force]= vrep.simxJointGetForce(clientID,jointHandle,option);
	if(rtn==vrep.simx_return_ok)
		jointTorque = force;
	else
		jointTorque = NaN;
	end

function success = commandJointAngle(jointHandle, angle)
	% Returns the joint angle for one joint in V-REP Nao.
	% 
	%   Outputs:
	%		
	%		success		Returns true is joint was able to be commanded, false
	%					otherwise.
	%
	%	Inputs:
	%
	%		jointHandle	V-REP handle for the requested joint.
	%
	%		angle 		Angle to command the joint to in radians.
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	% Get V-REP objects
	[clientID, vrep, option] = g_returnVREPobjects();

	% Set the joint angle target position
	rtn = vrep.simxSetJointTargetPosition(clientID,jointHandle,angle,option);
	if (rtn==vrep.simx_return_ok)
		success = true;
	else
		success = false;
	end

function success = setJointAngles(jointMap)
	% Commands all of the joints according to the angles in the joint map.
	% 
	%   Outputs:
	%		
	%		success		Returns false if ANY of the joints were not able to
	%					be commanded.
	%
	%	Inputs:
	%
	%		jointMap	Set of V-REP Nao's joint angles referenceable
	%					as RKneePitchJointAngle = jointMap('RKneePitch')
	%					in radians.
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	% Get handle map
	hMap = g_returnHandleMap();

	% Array of joint names
	jointNames = returnJointNames();

	% Initialize output
	success = true;

	% Command all of the joint angles
	for i = 1:length(jointNames)
		cmd_success = commandJointAngle(hMap(jointNames{i}), jointMap(jointNames{i}));
		if(cmd_success == false)
			success = false;
		end
	end

function jointMap = getJointAngles()
	% Gets all of the current joint angles for V-REP Nao.
	% 
	%   Outputs:
	%		
	%		jointMap	Set of V-REP Nao's joint angles referenceable
	%					as RKneePitchJointAngle = jointMap('RKneePitch')
	%					in radians.
	%
	%	Inputs:
	%
	%		<none>		Additional parameters are obtained via global functions.
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	% Get handle map
	hMap = g_returnHandleMap();

	% Array of joint names
	jointNames = returnJointNames();

	% Array for joint angles
	jointAngles = zeros(1, length(jointNames));

	% Get all of the joint angles
	for i = 1:length(jointNames)
		jointAngles(i) = readJointAngle(hMap(jointNames{i}));
	end

	% Produce the joint angle map
	jointMap = containers.Map(jointNames, jointAngles);

function torqueMap = getJointTorques()
	% Gets all of the current joint torques for V-REP Nao.
	% 
	%   Outputs:
	%		
	%		torqueMap	Set of V-REP Nao's joint torques referenceable
	%					as RKneePitchJointAngle = torqueMap('RKneePitch')
	%					in Nm.
	%
	%	Inputs:
	%
	%		<none>		Additional parameters are obtained via global functions.
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	% Get handle map
	hMap = g_returnHandleMap();

	% Array of joint names
	jointNames = returnJointNames();

	% Array for joint angles
	jointTorques = zeros(1, length(jointNames));

	% Get all of the joint angles
	for i = 1:length(jointNames)
		jointTorques(i) = readJointTorque(hMap(jointNames{i}));
	end

	% Produce the joint angle map
	torqueMap = containers.Map(jointNames, jointTorques);

%%% CRAWL FUNCTIONS %%%

function jointMap = crawl_ppArrayToJointMap(pp_Angles, pp_alpha)
	% Takes an array of joint angles according to the projected profile
	% crawling method and returns a joint map to be sent to V-REP Nao.
	% 
	%   Outputs:
	%		
	%		jointMap	Set of V-REP Nao's joint angles referenceable
	%					as RKneePitchJointAngle = jointMap('RKneePitch')
	%					in radians.
	%
	%	Inputs:
	%
	%		pp_Angles	Array of five joint angles prescribed from the 
	%					projected profile crawling algorithm. 
	%					pp_Angles = [theta1, theta2, theta3, theta4, theta5]
	%
	%		pp_alpha	Orientation of the projected profile elbow
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	% Array of joint names
	jointNames = returnJointNames();

	% Array for joint angles
	jointAngles = zeros(1, length(jointNames));

	% Produce the joint angle map
	jointMap = containers.Map(jointNames, jointAngles);

	%%% Map the projected profile angles to Nao %%%
	
	% Both ankles get theta2
	% Ankles are offset by 52 degrees (some tuning required/done)
	naoAnklePitch = pp_Angles(2) + deg2rad(60);
	jointMap('RAnklePitch') = naoAnklePitch;
	jointMap('LAnklePitch') = naoAnklePitch;

	% Both knees get theta3
	naoKneePitch = pp_Angles(3);
	jointMap('RKneePitch') = naoKneePitch;
	jointMap('LKneePitch') = naoKneePitch;

	% Both hips get theta4
	naoHipPitch = pp_Angles(4);
	jointMap('RHipPitch') = naoHipPitch;
	jointMap('LHipPitch') = naoHipPitch;

	% Both shoulders get theta5
	% Shoulders are offset by 90 degrees
	naoShoulderPitch = -pp_Angles(5) - pi/2;
	jointMap('RShoulderPitch') = naoShoulderPitch;
	jointMap('LShoulderPitch') = naoShoulderPitch;

	% Compute the arm angles

	% Elbow Yaw will be determined by IK
	% Configuration taken from old bRobot.cpp
	naoShoulderRoll = deg2rad(22);
	jointMap('RShoulderRoll') = -naoShoulderRoll;
	jointMap('LShoulderRoll') =  naoShoulderRoll;

	[naoElbowRoll, naoElbowYaw] = computeElbowAngles(pp_alpha);

	% naoElbowYaw = pi/2;
	jointMap('RElbowYaw') =  naoElbowYaw;
	jointMap('LElbowYaw') = -naoElbowYaw;

	% naoElbowRoll = deg2rad(87);
	jointMap('RElbowRoll') =  naoElbowRoll;
	jointMap('LElbowRoll') = -naoElbowRoll;

function [elbowRoll, elbowYaw] = computeElbowAngles(pp_alpha)
	% Takes the alpha angle according to the projected profile
	% crawling method and returns the elbow roll and yaw to be sent to V-REP Nao.
	% 
	%   Outputs:
	%		
	%		elbowRoll		The elbow roll angle in radians.
	%
	%		elbowYaw		The elbow yaw angle in radians.
	%
	%	Inputs:
	%
	%		pp_alpha	Orientation of the projected profile elbow
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	% Get the Nao parameters.
	[M, L] = returnNaoModelParameters();

	% The length of the humerus projected onto the z-x plane.
	lhw = L(5);
	% The length of the humerus.
	lh = L(6);

	% The distance from the shoulder to the elbow in the y-axis.
	d = sqrt(lh^2 - lhw^2);

	% The direction the forearm is required to point.
	vf = [1, 0, 0]';

	% The direction vector of the humerus, which is the axis of rotation of the elbow yaw.
	vh = [lhw*cos(pp_alpha), -d, lhw*sin(pp_alpha)]';
	vh = vh/lh;

	% The direction vector of the axis of the elbow roll, when elbow yaw is zero.
	nh = [-sin(pp_alpha), 0, cos(pp_alpha)]';

	% Compute elbow roll, the amount to rotate vh to get to vf.
	elbowRoll = acos(dot(vh, vf));

	%%% Compute elbow yaw, the amount to rotate the forearm around vh to align nh with u.
	% The direction vector needed that rotates vh onto vf.
	% u is the vector nh needs to be rotated to so vh can be
	% rotated onto vf.
	u = cross(vh, vf);
	u = u/norm(u);

	% Compute the angle betwwen nh and u.
	elbowYaw = acos(dot(nh, u));