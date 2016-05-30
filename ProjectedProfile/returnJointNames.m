
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
