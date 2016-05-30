function disconnectFromVREP(clientID, vrep)
	% Disconnects from the V-REP simulator.
	%	Outputs:
	%
	%		<none>   	Should only be called if there is a valid connection.
	%
	%   Inputs:
	%		
	%		clientID	Nao's V-REP client ID for communicating with the robot.
	%					Returns -1 if it could not connect to the simulator.
	%
	%		vrep 		Object for calling V-REP simulator functions. 
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	% Disconnect from client
	vrep.simxFinish(clientID);
	% Call destructor on vrep object
	vrep.delete(); 