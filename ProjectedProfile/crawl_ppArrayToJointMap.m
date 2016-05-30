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

	naoElbowYaw = pi/2;
	jointMap('RElbowYaw') =  naoElbowYaw;
	jointMap('LElbowYaw') = -naoElbowYaw;

	naoElbowRoll = deg2rad(87);
	jointMap('RElbowRoll') =  naoElbowRoll;
	jointMap('LElbowRoll') = -naoElbowRoll;
