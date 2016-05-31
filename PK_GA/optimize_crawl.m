% Path to joint torque table
[foldername, ~, ~] = fileparts(mfilename('fullpath'));
cd(foldername);
cd('..');
addpath(fullfile(pwd, 'Data'));

% Grab the torque data.
tq = load(fullfile(pwd, 'Data', 'vrep3_TorqueTable1_5deg.txt'));

% Global variables for function communication.
global X1 X2 X3 T1 T2 T3 T4 
global T1_interp T2_interp T3_interp T4_interp T1_interp_n T2_interp_n T3_interp_n T4_interp_n 
global x_init x_final t 
global alpha_min alpha_max theta3_min theta3_max theta4_min theta4_max

X1 = tq(:,6); % alpha
X2 = tq(:,3); % theta3
X3 = tq(:,4); % theta4
T1 = tq(:,11); % ankle pitch torque
T2 = tq(:,12); % knee pitch torque
T3 = tq(:,13); % hip pitch torque
T4 = tq(:,14); % shoulder pitch torque

% create torque interpolants
% use both nearest neighbor and linear interpolants (with linear preferred unless NaN)
T1_interp = TriScatteredInterp(X1, X2, X3, T1, 'linear');
T2_interp = TriScatteredInterp(X1, X2, X3, T2, 'linear');
T3_interp = TriScatteredInterp(X1, X2, X3, T3, 'linear');
T4_interp = TriScatteredInterp(X1, X2, X3, T4, 'linear');
T1_interp_n = TriScatteredInterp(X1, X2, X3, T1, 'nearest');
T2_interp_n = TriScatteredInterp(X1, X2, X3, T2, 'nearest');
T3_interp_n = TriScatteredInterp(X1, X2, X3, T3, 'nearest');
T4_interp_n = TriScatteredInterp(X1, X2, X3, T4, 'nearest');

% Set mins and maxs
alpha_min	= min(X1);
alpha_max  	= max(X1);
theta3_min  = min(X2);
theta3_max  = max(X2);
theta4_min  = min(X3);
theta4_max 	= max(X3);

% specified initial and final values of (\alpha, \theta_3, \theta_4)
x_init = [-30*pi/180 ; 0.28798  ;   0.47996];
x_final = [-90*pi/180 ; 0.28798  ;   0.47996];

% final time (normalized)
t_final = 1;

% sequence of times between 0 and t_final -- used to generate time sequence of angles
t_step = 0.01;
t = (0:t_step:t_final)';


% cubic spline coefficients -- for nominal straight line trajectories for (\alpha, \theta_3, \theta_4)
% each of (\alpha, \theta_3, \theta_4) corresponds to three coefficients (t^3, t^2, t)
x0 = [0;0;x_final(1)-x_init(1)  ;  zeros(6,1)];

% equality constraints for optimization
% angles at final time should equal x_final
Aeq = [ones(1,3),zeros(1,6)  ;  zeros(1,3),ones(1,3),zeros(1,3) ; zeros(1,6),ones(1,3)];
Beq = x_final - x_init;

% lower bounds and upper bounds on elements of x (the coefficients of the cubic splines)
lb = -2*ones(length(x0),1);
ub = 2*ones(length(x0),1);

% set options for the genetic algorithm
options = gaoptimset('PlotFcns', @gaplotbestf,'Display','iter');

% run the genetic algorithm to optimize the optimality cost implemented in optimize_crawl_fn
figure(1);
x = ga(@optimize_crawl_fn,9,[],[],Aeq,Beq,lb,ub,[],options)


% calculate time sequences of angle values for (\alpha, \theta_3, \theta_4)
% nominal trajectory
x1_0 = x0(1)*t.^3 + x0(2)*t.^2 + x0(3)*t + x_init(1);
x2_0 = x0(4)*t.^3 + x0(5)*t.^2 + x0(6)*t + x_init(2);
x3_0 = x0(7)*t.^3 + x0(8)*t.^2 + x0(9)*t + x_init(3);
% optimized trajectory
x1_ = x(1)*t.^3 + x(2)*t.^2 + x(3)*t + x_init(1);
x2_ = x(4)*t.^3 + x(5)*t.^2 + x(6)*t + x_init(2);
x3_ = x(7)*t.^3 + x(8)*t.^2 + x(9)*t + x_init(3);

% plot nominal and optimized trajectories
figure(2);
set(0,'defaultaxesfontsize',14);
line_width=2;
font_size=14;
subplot(3,1,1); plot(t,x1_0,'b',t,x1_,'g','LineWidth',line_width); xlabel('t (s)','FontSize',font_size); ylabel('\alpha (rad)','FontSize',font_size);
subplot(3,1,2); plot(t,x2_0,'b',t,x2_,'g','LineWidth',line_width); xlabel('t (s)','FontSize',font_size); ylabel('\theta_3 (rad)','FontSize',font_size);
subplot(3,1,3); plot(t,x3_0,'b',t,x3_,'g','LineWidth',line_width); xlabel('t (s)','FontSize',font_size); ylabel('\theta_4 (rad)','FontSize',font_size);

% calculate optimization costs for nominal and optimized trajectories
x0_cost = optimize_crawl_fn(x0) * t_step
x_cost = optimize_crawl_fn(x) * t_step


% best x value found after a few runs of the genetic algorithm
x_best = [ -1.9362   -0.8890    1.7781   -0.0872   -0.0099    0.0972    0.3388   -0.6390    0.3002];
c_best = optimize_crawl_fn(x_best) * t_step
