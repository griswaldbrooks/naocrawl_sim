function [clientID, vrep] = connectToVREP()
	% Connects to the V-REP simulator.
	% 
	%   Outputs:
	%		
	%		clientID	Nao's V-REP client ID for communicating with the robot.
	%					Returns -1 if it could not connect to the simulator.
	%
	%		vrep 		Object for calling V-REP simulator functions. 
	%
	%	Inputs:
	%
	%		<none>   	Function uses a fixed set of configuration parameters.
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	% Get the vrep object using the prototype file (remoteApiProto.m)
	vrep=remApi('remoteApi');
	% In case there are any other open connections, close them.
	vrep.simxFinish(-1);
	% Get the client ID for Nao, assuming it is running on this computer (127.0.0.1)
	% and the V-REP child script is running on port number 19999
	clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

	% Check if the connection was made successfully
	try
		% If a valid client ID was returned
		if (clientID>-1)
			% Test V-REP connection
			[res,objs]=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_oneshot_wait);
			if (res~=vrep.simx_return_ok)
				% Connection failed, close connection, call vrep destructor
				vrep.simxFinish(clientID);
				vrep.delete();
				% Set client ID to -1 so the calling function can know the connection failed
				clientID = -1;
			end
		else
			% Couldn't get valid client ID, call vrep destructor
			vrep.delete();
		end
		
	catch err
		 % Close the line if still open
		vrep.simxFinish(clientID);
		% Call the destructor!
		vrep.delete(); 
		% Set client ID to -1 to inform calling function of error
		clientID = -1;
	end;
	
