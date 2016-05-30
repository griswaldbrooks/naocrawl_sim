function generateJointTorqueTable(clientID, vrep)
	% Uses V-REP to generate a csv file of joint angles and their corresponding
	% joint torques for Nao's crawling gait. Simulation must be running with the
	% V-REP Nao laying on the ground. Function starts with alpha = -30 and goes to
	% alpha = -90
	% 
	%   Outputs:
	%		
	%		<none>			CSV File called torqueTable.txt saved in the temp
	%						directory. Each row is of the format:
	%						[   theta1, 	theta2,		theta3,		theta4,		theta5, 	alpha,...	
	%							RAnklePitchAngle,   RKneePitchAngle, 	RHipPitchAngle, 	RShoulderPitchAngle, ...
	%							RAnklePitchTorque,  RKneePitchTorque, 	RHipPitchTorque, 	RShoulderPitchTorque]
	%
	%	Inputs:
	%
	%		clientID   		Nao's V-REP client ID returned from	vrep.simxStart.
	%
	%		vrep 			Object for calling V-REP simulator functions. 
	%						Returned from the remApi function.
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com
	
	% Initialize global parameters
	if(g_initGlobalParameters(clientID, vrep))

		% Iterate through all alpha from -30:-5:-90
		% Iterate through all theta3 from -5: 5: 45
		% Iterate through all theta4 from 27.5:-5:-30

		% Build the configuration lists
		% Try increments of one degree
		alpha = -30:-2.5:-90;
		theta3 = -5:2.5:45;
		theta4 = 27.5:-2.5:-30;

		% Total number of iterations
		totalIterations = length(alpha)*length(theta3)*length(theta4);
		% Iterator for which row in the file we are at
		fileRowItr = 1;


		% Matrix for recording torques
		recordedTorques = zeros(totalIterations, 14);

		% Create waitbar
    	waitHandle = waitbar(0,'Loading parameters','Name','Iterating through joint configurations',...
            'CreateCancelBtn','setappdata(gcbf,''canceling'',1)');
    	setappdata(waitHandle,'canceling',0);

    	% Cancel iteration flag
    	cancelIterations = 0;

    	% Amount of time to pause for to allow joints to settle
    	dt = 0.5;

		% Iterate through all of the joints
        for alpha_itr = 1:length(alpha)
            for theta3_itr = 1:length(theta3)
                for theta4_itr = 1:length(theta4)

                    % Check for Cancel button press
	                if getappdata(waitHandle,'canceling')
	                	cancelIterations = 1;
	                end

	                % Get the Projected Profile joint configuration
	                theta_cfg = degtorad([theta3(theta3_itr), theta4(theta4_itr), alpha(alpha_itr)]');
	                q = returnConstrainedJointConfiguration(theta_cfg);

	                % Compute new joint angles
	                jointMap = crawl_ppArrayToJointMap(q, alpha);

	                % Set V-REP Nao joints
	                jointAnglesSentSuccessfully = setJointAngles(jointMap);

	                if (~jointAnglesSentSuccessfully)
		                disp(['Joint angles sent unsuccessfully. [Alpha, Theta3, Theta4] ', ...
                            num2str([alpha(alpha_itr), theta3(theta3_itr), theta4(theta4_itr)])]);
	                end

	                % Show animation
	                generateAnimation(q', 1, false);

	                % Allow joints to settle
	                pause(dt);		

	                % Read joint torques
	                torqueMap = getJointTorques();

	                % Record torque values
	                recordedTorques(fileRowItr,:) = [	q', degtorad(alpha(alpha_itr)), ...
	                                        jointMap('RAnklePitch'),jointMap('RKneePitch'),jointMap('RHipPitch'),jointMap('RShoulderPitch'), ...
	                                        torqueMap('RAnklePitch'),torqueMap('RKneePitch'),torqueMap('RHipPitch'),torqueMap('RShoulderPitch')];

	                % Display the waitbar
	                waitbar(fileRowItr/totalIterations, waitHandle, ...
                        sprintf('Alpha: %d Theta3: %d Theta4: %d', alpha(alpha_itr), theta3(theta3_itr), theta4(theta4_itr)));

	                % Increment iterator
	                fileRowItr = fileRowItr + 1;

	                % Check for cancellation flag
	                if (cancelIterations == 1)
	                	break;
	                end
                
                end
                
                % Check for cancellation flag
                if (cancelIterations == 1)
                    break;
                end
            end
            
            % Check for cancellation flag
            if (cancelIterations == 1)
                break;
            end
        end

		% Close waitbar
		delete(waitHandle);
        

		% Save torque values to csv file
		csvwrite('temp/vrepn_TorqueTablem.txt', recordedTorques);
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

function jointNames = returnJointNames()
	% Returns a cell array of the strings used as joint names for
	% the joint maps.
	% 
	%   Outputs:
	%		
	%		jointNames	Cell array of joint name strings
	%
	%	Inputs:
	%
	%		<none>		List of joint names is fixed.
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	% Array of joint handle names
	jointNames = {	'RHipYawPitch'
				    'RHipRoll'
				    'RHipPitch'
				    'RKneePitch'
				    'RAnklePitch'
				    'LHipYawPitch'
				    'LHipRoll'
				    'LHipPitch'
				    'LKneePitch'
				    'LAnklePitch'
				    'RShoulderPitch'
				    'RShoulderRoll'
				    'RElbowYaw'
				    'RElbowRoll'
				    'LShoulderPitch'
				    'LShoulderRoll'
				    'LElbowYaw'
				    'LElbowRoll'};

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
	[rtn, force]= vrep.simxJointGetForce(clientID,jointHandle,option);
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
