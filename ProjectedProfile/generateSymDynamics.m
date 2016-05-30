function [D, C, V, J, H, A] = generateSymDynamics()
% Wrapper function that uses the DH-SYM library functions to 
% compute the symbolic dynamic and kinematic matrices for the 
% Nao robot for the projected profile crawling algorithm, with
% masses concentrated at the joints
% 
% Outputs:
%		
%		D 		N x N mass matrix
%
%		C		N x N coriolis matrix
%
%		V		N x 1 gravity vector
%
%		J 		N x 1 cell of Jacobian matrices. J{end} contains the jacobian of 
% 				the full kinematic chain.
%
%		H 		(N+1) x 1 cell of homogeneous transformations to each i-th link
% 				from the base of the kinematic chain.
%
%		A 		N x 1 cell of homogeneous transformations from each link. Used
% 				to form each successive matrix in "H."
%
%	Inputs:
%
%		<none> 	This function is only for this one configuration as a wrapper. 
%				To make it configurable would be somewhat redundant as the DH-SYM
%				does this.
%
% Griswald Brooks
% griswald.brooks@gmail.com

% 5 Revolute Joint Manipulator
config  = 'RRRRR'; 
table   = [   
            sym('[L1,   0,  0,  q1(t)]'); ...
            sym('[L2,   0,  0,  q2(t)]'); ...
            sym('[L3,   0,  0,  q3(t)]'); ...
            sym('[L4,   0,  0,  q4(t)]'); ...
            sym('[L5,   0,  0,  q5(t)]') ...
        ];
% Gravity pointing down
gravity = sym('[0;1;0]');

% Generate the manipulator matrices
[D, C, V, J, H, A] = dh2dyn(table,config,gravity);