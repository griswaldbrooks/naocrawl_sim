function plotArm2(H, armcolor)
% Take cell array of transformation matrices for a robot manipulator and 
% plot them.
%
%   Inputs:
%	H           (N+1) x 1 cell array of numerically evaluated homogeneous
%               transformation matrices for a robot manipulator. Each
%               element is a 4x4 matrix.
%
%   armcolor    String containing color of the links
%
% Griswald Brooks
% griswald.brooks@gmail.com

% Number of joints
k = size(H,1);
% Frame scaling
fr_scale = 2;
% Vector for grabbing origin
d = [0,0,0,1]';

% Joint vector holds the points of each joint in the following form:
% joint_p = [px, py, pz, 1]
joint_p = zeros(k,4);
for i = 1:k
    % Position of each joint
    joint_p(i,:) = (H{i}*d)';

    % Plot Joint Positions
    plot3(joint_p(i,1),joint_p(i,2),joint_p(i,3),'bo');
    
    % Plot Frames
    plotFrame3(H{i}, fr_scale, 3);
end

% Plot Links
for i = 1:(k-1)
    line([joint_p(i,1),joint_p(i+1,1)],[joint_p(i,2),joint_p(i+1,2)],[joint_p(i,3),joint_p(i+1,3)],'Color',armcolor, 'LineWidth', 5);
end

% Label Axes
xlabel('X axis, in');
ylabel('Y axis, in');
zlabel('Z axis, in');
