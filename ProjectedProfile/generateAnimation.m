function generateAnimation(q, t, USER_ITERATED)
    % Take robot joint angles and display the robot's projected profile
    % kinematic configuration.
    %
    %   Outputs:
    %
    %       <none>          Robot is displayed in a plot.
    %
    %   Inputs:
    %
    %       q               M x N sequence joint angles (radians).
    %                       q(k, :) = [theta1, theta2, theta3, theta4, theta5]
    %
    %       t               M x 1 Vector of time elements for each state.
    %
    %       USER_ITERATED   Flag that controls if user has to hit enter to 
    %                       advance animation or if it is done automatically.
    %
    % Griswald Brooks
    % griswald.brooks@gmail.com

    % Get link lengths for Nao
    [~, L] = returnNaoModelParameters();

    % Width of the plotted lines
    line_width = 3; 

    % Size of the plotted joints
    scale = 20;

    % Vector for grabbing position vector
    d = [0,0,1]';

    % Make a figure
    figure(1)

    % Iterate through all of the joints
    for k = 1:length(t)
        % Start the timer on the plotting function
        tic

        % Clear axis before plotting
        cla

        % Set axis
        axis([-30 460 -30 160]);
        
        % Compute orientation of elbow (end effector)
        alpha = sum(q(k,:));

        % Get Transformation Matrices from n to n+1
        H1 = DH(q(k,1), L(1));
        H2 = DH(q(k,2), L(2));
        H3 = DH(q(k,3), L(3));
        H4 = DH(q(k,4), L(4));
        H5 = DH(q(k,5), L(5));

        % Get Transformation Matrices from origin to n
        T_ankle = H1;
        T_knee = T_ankle*H2;
        T_hip = T_knee*H3;
        T_shoulder = T_hip*H4;
        T_elbow = T_shoulder*H5;

        % Grab position vectors for each joint
        ankle_p = T_ankle*d;
        knee_p = T_knee*d;
        hip_p = T_hip*d;
        shoulder_p = T_shoulder*d;
        elbow_p = T_elbow*d;


        % Plot Joint Positions
        plot(0,0,'bo');
        plot(ankle_p(1),ankle_p(2),'bo');
        plot(knee_p(1),knee_p(2),'bo');
        plot(hip_p(1),hip_p(2),'bo');
        plot(shoulder_p(1),shoulder_p(2),'bo');
        plot(elbow_p(1),elbow_p(2),'bo');

        % Plot Links
        line([0,ankle_p(1)],[0,ankle_p(2)],'Color','k','LineWidth',line_width);
        line([ankle_p(1),knee_p(1)],[ankle_p(2),knee_p(2)],'Color','k','LineWidth',line_width);
        line([knee_p(1),hip_p(1)],[knee_p(2),hip_p(2)],'Color','k','LineWidth',line_width);
        line([hip_p(1),shoulder_p(1)],[hip_p(2),shoulder_p(2)],'Color','k','LineWidth',line_width);
        line([shoulder_p(1),elbow_p(1)],[shoulder_p(2),elbow_p(2)],'Color','k','LineWidth',line_width);

        % Plot Frames
        plotFrame(eye(3),scale,line_width);
        plotFrame(H1, scale,line_width);
        plotFrame(H1*H2, scale,line_width);
        plotFrame(H1*H2*H3, scale,line_width);
        plotFrame(H1*H2*H3*H4, scale,line_width);
        plotFrame(H1*H2*H3*H4*H5, scale,line_width);
        xlabel('X axis (mm)');
        ylabel('Z axis (mm)');
        title_text1 = 'Simplified Kinematic Model, \alpha = ';
        title_text2 = num2str((180/pi)*alpha);
        title_text3 = '\circ';
        title_text = [title_text1,title_text2,title_text3];
        title(title_text,'interpreter','tex');

        % Report alpha - theta5 angle and wait for 
        string_alpha_theta5 = num2str(rad2deg(alpha - q(k,5)));
        text(375,140,['\alpha - \theta_5 = ',string_alpha_theta5]);

        % Display time
        text(375,120,['Time (sec): ', num2str(t(k))]);

        % User iterated or automatic?
        if USER_ITERATED
            input('Press enter to display next configuration.');
        else
            % Find the dt of the time signal
            if (length(t) == 1)
                dt = 0;
            elseif (k == length(t))
                dt = t(k) - t(k-1);
            else
                dt = t(k+1) - t(k);
            end
            
            % Grab the time it took to plot
            plottingTime = toc;

            % Adjust dt to compensate for plotting time
            if (dt > plottingTime)
                dt = dt - plottingTime;
            end

            % Pause animation to make it plot close to the actual
            % moving speed
            pause(dt);

        end
    end
