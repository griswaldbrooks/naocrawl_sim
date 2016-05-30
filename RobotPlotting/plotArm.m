function endEff_p = plotArm(H, q, L, armcolor)
% Take Robot Manipulator cell array of symbolic H matrices generated from 
% the DH-SYM library with their numeric parameters q and L, and plots the
% robot arm in three dimensions (R3).
%
%   Outputs:
%		
%		endEff_p    Position of the end effector in R3.
%
%	Inputs:
%
%       H           (N+1) x 1 cell of homogeneous transformations to each i-th 
%                   link from the base of the kinematic chain.
%
%       q           N x 1 numeric vector of joint angles
%
%       L           N x 1 numeric vector of link lengths
%
%       armcolor    String containing color of the links
%
% Griswald Brooks
% griswald.brooks@gmail.com

% Number of transformation matrices in H
k = size(H,1);
% Frame scaling
fr_scale = 2;

% Vector for grabbing origin
d = [0,0,0,1]';

% Numerically evaluate all of the H matrices
He = evaluateH(H,q,L);

% Joint vector holds the points of each joint in the following form:
% joint_p(i) = [px, py, pz, 1]
joint_p = zeros(k,4);
for i = 1:k
    % Position of each joint
    joint_p(i,:) = (He{i}*d)';

    % Plot Joint Positions
    plot3(joint_p(i,1),joint_p(i,2),joint_p(i,3),'bo');
    
    % Plot Frames
    plotFrame3(He{i}, fr_scale, 3);
end

% Plot Links
for i = 1:(k-1)
    line([joint_p(i,1),joint_p(i+1,1)],[joint_p(i,2),joint_p(i+1,2)],[joint_p(i,3),joint_p(i+1,3)],'Color',armcolor, 'LineWidth', 5);
end

% Label Axes
xlabel('X axis, in');
ylabel('Y axis, in');
zlabel('Z axis, in');

% Grab end effector point
endEff_p = joint_p(k,1:3)';

function He = evaluateH(H, q, L)
% Take Robot Manipulator cell array of symbolic H matrices generated from 
% the DH-SYM library with their numeric parameters q and L, and numerically
% evaluate them
%
%   Outputs:
%		
%		He      (N+1) x 1 cell array of numerically evaluated H matrices
%
%	Inputs:
%
%       H 		(N+1) x 1 cell array of symbloic homogeneous 
%               transformations to each i-th link from the base of the 
%               kinematic chain.
%
%       q       N x 1 numeric vector of joint angles
%
%       L       N x 1 numeric vector of link lengths

% Check that a column vector has been passed in
if  size(q,1) < size(q,2)           || ...
    size(L,1) < size(L,2)           
        
    error('myApp:argChk', 'Parameter vectors must be column vectors.')
    
else

    % Create output cell array
    He = cell(size(H));
    % Number of matrices to evaluate
    k = size(H,1);
    % Number of joint variables
    n = size(q,1);
   
    % Create new variables q1, q2, etc. and set them to q(1), q(2), etc.
    % L1, L2, etc. and set them to l(1), l(2), etc.
    for i = 1:n
        eval([genvarname(['q',num2str(i)]),     '= q(i);']);
        eval([genvarname(['L',num2str(i)]),     '= L(i);']);
    end

    % Numerically evaluate matrices
    for i = 1:k
        He{i} = vpa(subs(H{i}));
    end

end