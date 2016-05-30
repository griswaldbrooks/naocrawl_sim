function [c]  = optimize_crawl_fn(x)
  % Global variables for function communication.
  global X1 X2 X3 T1 T2 T3 T4 
  global T1_interp T2_interp T3_interp T4_interp T1_interp_n T2_interp_n T3_interp_n T4_interp_n 
  global x_init x_final t 
  global alpha_min alpha_max theta3_min theta3_max theta4_min theta4_max

  % Projected Profile Library
  addpath(genpath(fullfile('..','ProjectedProfile')));
  % Library for plotting robots in MATLAB (has Dh function)
  addpath(genpath(fullfile('..','RobotPlotting')));

  % calculate time sequence of angle values for (\alpha, \theta_3, \theta_4)
  % corresponding to cubic spline coefficients given in x
  x1_ = x(1)*t.^3 + x(2)*t.^2 + x(3)*t + x_init(1);
  x2_ = x(4)*t.^3 + x(5)*t.^2 + x(6)*t + x_init(2);
  x3_ = x(7)*t.^3 + x(8)*t.^2 + x(9)*t + x_init(3);

  % optimize torque values at the computed angle values
  % use both nearest neighbor and linear interpolants
  % prefer linear unless interpolated value is NaN (i.e., if angles outside convex hull of provided empirical data used for interpolation)
  T1_ = T1_interp(x1_,x2_,x3_);
  T2_ = T2_interp(x1_,x2_,x3_);
  T3_ = T3_interp(x1_,x2_,x3_);
  T4_ = T4_interp(x1_,x2_,x3_);
  T1_n = T1_interp_n(x1_,x2_,x3_);
  T2_n = T2_interp_n(x1_,x2_,x3_);
  T3_n = T3_interp_n(x1_,x2_,x3_);
  T4_n = T4_interp_n(x1_,x2_,x3_);
  T1_(isnan(T1_)) = T1_n(isnan(T1_));
  T2_(isnan(T2_)) = T2_n(isnan(T2_));
  T3_(isnan(T3_)) = T3_n(isnan(T3_));
  T4_(isnan(T4_)) = T4_n(isnan(T4_));

  % Calculate optimization cost
  % c = sum(sum(T1_.^2 + T2_.^2 + T3_.^2 + 5*T4_.^2));
  
  % Sign constraint on alpha
  % Min height of hips
  % Constraint on delta alpha
  
  % Time sequence of hip heights.
  hip_height = zeros(length(x1_), 1);
  for i = 1:length(hip_height)
    % Get the full joint configuration q for the constraint configuration [theta_3, theta_4, alpha].
    q = returnConstrainedJointConfiguration([x2_(i), x3_(i),x1_(i)]);

    %%% Get the height of the hip joint. %%%
    % Get link lengths for Nao
    [~, L] = returnNaoModelParameters();
    % Get Transformation Matrices from n to n+1.
    H1 = DH(q(1), L(1));
    H2 = DH(q(2), L(2));
    H3 = DH(q(3), L(3));

    % Get Transformation Matrices from origin to n.
    T_ankle = H1;
    T_knee = T_ankle*H2;
    T_hip = T_knee*H3;

    % Vector for grabbing position vector
    d = [0,0,1]';

    % Grab position vectors for each joint.
    hip_p = T_hip*d;

    % Grab the hip height.
    hip_height(i) = hip_p(2);
  end

  % Sum up all of the torques incurred in the time sequence but weight T4 (the shoulder joint)
  % more heavily because it is the weakest joint.
  cTorque = sum(sum(T1_.^2 + T2_.^2 + T3_.^2 + 5*T4_.^2));

  % Check to see if alpha ever backtracks (has a positive velocity).
  COST_ALPHA = 1e5;
  cAlpha = COST_ALPHA*sum(diff(x1_) > 0);

  % Ensure that the height of the hips don't hit the floor.
  HIP_HEIGHT_THRESH = 40; % in mm
  COST_HIP_HEIGHT = 1e5;
  cHeight = COST_HIP_HEIGHT*sum(hip_height < HIP_HEIGHT_THRESH);

  % Enforce joint limits.
  COST_JOINT_LIMIT = 1e5;
  alphaViolations = sum(x1_ < alpha_min) + sum(x1_ > alpha_max);
  theta3Violations = sum(x2_ < theta3_min) + sum(x2_ > theta3_max);
  theta4Violations = sum(x3_ < theta4_min) + sum(x3_ > theta4_max);
  cLimits = COST_JOINT_LIMIT*(alphaViolations + theta3Violations + theta4Violations);

  % Sum all of the costs.
  c = cTorque + cAlpha + cHeight + cLimits;

end
  
