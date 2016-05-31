function [M, L] = returnNaoModelParameters()
	% Returns masses and link lengths for Nao used in projected profile model.
	% 
	%   Outputs:
	%		
	%		M   		N x 1 vector of point masses located at the joints in kg
	%
	%		L   		N x 1 vector of link lengths in mm.
	%
	%	Inputs:
	%
	%		<none>   	Configuration is only applicable to the projected profile
	%					model used in the context of crawl gaiting.
	%
	% Griswald Brooks
	% griswald.brooks@gmail.com

	%%% Robot Parameters taken from Nao V4 Documentation 
	% Masses (kg):

	% Head: 
	m_head = 0.60;
	% Neck: 
	m_neck = 0.06;
	% Torso : 
	m_torso = 1.05;

	% One Arm: 
	%	Shoulder Joint: 
	m_sj = 0.07;
	%	Bicep:	
	m_bicep = 0.16;
	%	Elbow:
	m_elbow = 0.06;
	%	Forearm:
	m_forearm = 0.08;
	%	Hand:
	m_hand = 0.19;

	% One Leg: 
	%	Pelvis: 
	m_pelvis = 0.07;
	%	Hip: 
	m_hip = 0.13;
	%	Thigh: 
	m_thigh = 0.39;
	%	Tibia: 
	m_tib = 0.30;
	%	Ankle: 
	m_ank = 0.13;
	%	Foot: 
	m_foot = 0.16;

	% Mass distributed as point masses at the joints
	M = zeros(5,1);
	M(1) = 2*(m_foot + m_ank + 0.5*m_tib);
	M(2) = 2*(0.5*m_tib + 0.5*m_thigh);
	M(3) = 2*(0.5*m_thigh + m_hip + m_pelvis) + 0.25*m_torso;
	M(4) = 0.75*m_torso + 2*(m_sj + 0.5*m_bicep) + m_neck + m_head;
	M(5) = 2*(0.5*m_bicep + m_elbow + m_forearm + m_hand);

	% Link Lengths of the Nao in mm
	% Body Length is calculated as 
	% body_length = -Neck-to-shoulder + NeckOffsetZ + HipOffsetZ
	% from the Nao documentation
	body_length = -23 + 126.5 + 85;
	thigh_length = 100;
	humerus_length = 105;
	% Length of humerus when projected onto the saggital plane
	projected_humerus_length = 89.75;
	tibia_length = 102.9;

	% Foot length in this context is calculated as the line from 
	% the ankle to tip of foot, FootHeight in the Nao documentation
	% and the measured sole length
	foot_length = sqrt(45.19^2 + 93.5^2);

	% Assign the link lengths to the length vector
	L = zeros(5,1);
	L(1) = foot_length;
	L(2) = tibia_length;
	L(3) = thigh_length;
	L(4) = body_length;
	L(5) = projected_humerus_length;
	L(6) = humerus_length;