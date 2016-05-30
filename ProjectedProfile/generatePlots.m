function generatePlots()

% Plot joint torques and save plot to file
vrepTorques = csvread('temp/vrepn_Torquesm.txt',1,0);

figure(1)
plot(vrepTorques)
title('VREP Torques')
xlabel('Iteration')
ylabel('Torque (Nm)')
legend('Theta2','Theta3','Theta4','Theta5')
print('-dpng','-r72','temp/VREPn_Torquesm.png')

% 	% Plot error graphs
% e_q        = zeros(n, length(t));
% e_q_dot    = zeros(n, length(t));
% e_q_ddot   = zeros(n, length(t));

% for i = 1:length(t)
%     e_q(:,i)        = x(i,1:n)' - Xref(:,i);
%     e_q_dot(:,i)    = x(i,n+1:2*n)' - Xref_dot(:,i);
%     e_q_ddot(:,i)   = x(i,2*n+1:3*n)' - Xref_ddot(:,i);
% end

% % Print variables
% res = '-r72';
% format = '-dpng';
% lnWidth = 2;
% legendFontSize = 12;
% labelFontSize = 12;
% titleFontSize = 14;

% figure(1)
% cla
% hold all
% axis square
% i = 10;
% % Plot simulation
% plotArm2(Hq{i}, 'k');
% % Plot reference
% plotArm2(Href{i},'r');

% Ymax = max_dim;
% Ymax = 10*ceil(Ymax/10); % Ceil to nearest 10
% Ymin = 0;
% Xmax = max_dim;
% Xmin = 0;
% % Ymin = 50*floor(Ymin/50); % Floor to nearest 10
% axis([Xmin, Xmax, Ymin, Ymax])
% % hLegend = legend('Controlled', 'Reference');
% hTitle = title('Reference Manipulator and Contolled Manipulator Overlay');
% hXLabel = xlabel('X Axis (m)');
% hYLabel = ylabel('Y Axis (m)');

% % Figure settings
% % set([hLegend, gca]             , ...
% %     'FontSize'   , legendFontSize           );
% set([hXLabel, hYLabel]  , ...
%     'FontSize'   , labelFontSize          );
% set( hTitle                    , ...
%     'FontSize'   , titleFontSize          , ...
%     'FontWeight' , 'bold'      );

% set(gca, ...
%   'Box'         , 'off'     , ...
%   'TickDir'     , 'out'     , ...
%   'TickLength'  , [.02 .02] , ...
%   'XMinorTick'  , 'off'      , ...
%   'YMinorTick'  , 'off'      , ...
%   'XColor'      , [.3 .3 .3], ...
%   'YColor'      , [.3 .3 .3], ...
%   'YTick'       , Ymin:5:Ymax, ...
%   'LineWidth'   , 1         );

% print(format,res,'arm_overlay1');

% figure(2)
% cla
% plot(t,radtodeg(e_q(1,:)), t,radtodeg(e_q(2,:)));
% Ymax = radtodeg(max(max(e_q)));
% Ymax = 50*ceil(Ymax/50); % Ceil to nearest 10
% Ymin = radtodeg(min(min(e_q)));
% Ymin = 50*floor(Ymin/50); % Floor to nearest 10
% axis([0, t(end), Ymin, Ymax])
% hLegend = legend('Joint 1', 'Joint 2');
% hTitle = title('Joint Angle Errors');
% hXLabel = xlabel('Time (t)');
% hYLabel = ylabel('Angle Error (Degrees)');

% % Figure settings
% set([hLegend, gca]             , ...
%     'FontSize'   , legendFontSize           );
% set([hXLabel, hYLabel]  , ...
%     'FontSize'   , labelFontSize          );
% set( hTitle                    , ...
%     'FontSize'   , titleFontSize          , ...
%     'FontWeight' , 'bold'      );

% set(gca, ...
%   'Box'         , 'off'     , ...
%   'TickDir'     , 'out'     , ...
%   'TickLength'  , [.02 .02] , ...
%   'XMinorTick'  , 'off'      , ...
%   'YMinorTick'  , 'off'      , ...
%   'XColor'      , [.3 .3 .3], ...
%   'YColor'      , [.3 .3 .3], ...
%   'YTick'       , Ymin:10:Ymax, ...
%   'LineWidth'   , 1         );

% print(format,res,'fbl_robust_error_pos_unk_mass1');

% figure(3)
% cla
% plot(t,radtodeg(e_q_dot(1,:)), t,radtodeg(e_q_dot(2,:)));
% Ymax = radtodeg(max(max(e_q_dot)));
% Ymax = 100*ceil(Ymax/100); % Ceil to nearest 10
% Ymin = -radtodeg(max(max(e_q_dot)));
% Ymin = 100*floor(Ymin/100); % Floor to nearest 10
% % Ymin = radtodeg(min(min(e_q_dot)));
% axis([0, t(end), Ymin, Ymax])
% hLegend = legend('Joint 1', 'Joint 2');
% hTitle = title('Joint Velocity Errors');
% hXLabel = xlabel('Time (t)');
% hYLabel = ylabel('Velocity Error (Degrees/Second)');

% % Figure settings
% set([hLegend, gca]             , ...
%     'FontSize'   , legendFontSize           );
% set([hXLabel, hYLabel]  , ...
%     'FontSize'   , labelFontSize          );
% set( hTitle                    , ...
%     'FontSize'   , titleFontSize          , ...
%     'FontWeight' , 'bold'      );

% set(gca, ...
%   'Box'         , 'off'     , ...
%   'TickDir'     , 'out'     , ...
%   'TickLength'  , [.02 .02] , ...
%   'XMinorTick'  , 'off'      , ...
%   'YMinorTick'  , 'off'      , ...
%   'XColor'      , [.3 .3 .3], ...
%   'YColor'      , [.3 .3 .3], ...
%   'YTick'       , Ymin:50:Ymax, ...
%   'LineWidth'   , 1         );

% print(format,res,'fbl_robust_error_vel_unk_mass1');