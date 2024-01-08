classdef VirtualRobot < handle

    properties (Constant)

        joint_limits = [-pi/4, pi/4;
            -pi/4, pi/4;
            -(14/8)*pi, (14/8)*pi;
            -(5/6)*pi, (5/6)*pi];

        % Number of positions the trajectory holds (circular buffer)
        trajectoryLength = 30;

         % Define a condition number for the Jacobian above when the
         % configuration is considered a singularity configuraion
         % cond(J) > singularityThreshold --> checkSingularity = true;
        singularityThreshold = 25;
        
        % Small change in joint angles for numeric differentiation of
        % forward kinematics for jacobian calculation
        delta_q = 1e-6;

    end

    properties
        links   % An array of Link objects, defining the physical connections between joints
        frames % An array of Frame objects, defining joints and frames
        fig % The figure in which everything is visualized
        workspace % A workspace object

        trajectoryPlot % Handle for the trajectory plot
        trajectoryIndex = 1; % Current index in the trajectory buffer
        trajectoryPoints;
        tcpTextHandle % Handle for the end-effector position text

        q = [0;0;0;0]; % Joint Angles

    end

    methods

        %% Constructor
        function obj = VirtualRobot

            %% Setup Frames and Joints of the simulated robot
            orig_frame = CustomFrame([0; 0; 0], [], 'Origin', []);
            joint1 = CustomFrame([0; 0; 94.5127], orig_frame, 'Joint 1', 'y');
            joint2 = CustomFrame([0;0;0], joint1, 'Joint 2', 'x');
            joint3 = CustomFrame([0;0;150.0277], joint2, 'Joint 3', 'z');
            joint4 = CustomFrame([0;0;157.8995], joint3, 'Joint 4', 'x');
            endeffector_frame = CustomFrame([0;0;220.40], joint4, 'Endeffector', []);

            %% Setup Links of the simulated robot
            numLinks = 5; % Number of links
            grayLevels = linspace(0.2, 0.8, numLinks);  % Define a range of grayscale values. Start from 0.2 (dark) to 0.8 (lighter)

            link1 = CustomLink(orig_frame, joint1, repmat(grayLevels(1), 1, 3));  % Link 1 with first grayscale value
            link2 = CustomLink(joint1, joint2, repmat(grayLevels(2), 1, 3));  % Link 2 with second grayscale value
            link3 = CustomLink(joint2, joint3, repmat(grayLevels(3), 1, 3));  % and so on...
            link4 = CustomLink(joint3, joint4, repmat(grayLevels(4), 1, 3));
            link5 = CustomLink(joint4, endeffector_frame, repmat(grayLevels(5), 1, 3));

            obj.links = [link1, link2, link3, link4, link5];
            obj.frames = [orig_frame, joint1, joint2, joint3, joint4, endeffector_frame];

            %% Create the workspace object
            obj.workspace = Workspace(obj);

        end

        %% Logic
        function q = getQ(obj)
            q = obj.q;
        end

        function setQ(obj, q)

            q = mod(q + 2*pi, 4*pi) - 2*pi;  % Convert the desired q to the range [-2pi, 2pi]
            for i = 1:4
                obj.frames(i+1).setAngle(q(i));
                obj.q(i) = q(i);
            end

        end

        function [g_r_EE] = getEndeffectorPos(obj)
            g_r_EE = obj.frames(end).getGlobalPosition;
        end

        function J = getJacobianNumeric(obj)
            % Computes the Jacobian matrix numerically, relating joint velocities to end-effector velocities.
            % Uses finite differences on forward kinematics by perturbing joint angles with 'delta_q'.
            % Numerical methods may offer speed advantages over symbolic ones, but precision can vary.


            % Initialize Jacobian matrix
            J = zeros(3, 4);

            q_local = obj.getQ;

            %Save q to restore it later
            q_save = q_local;

            % For each joint angle
            for i = 1:4
                % Perturb joint angle i
                q_plus = q_local;
                q_plus(i) = q_plus(i) + obj.delta_q;
                q_minus = q_local;
                q_minus(i) = q_minus(i) - obj.delta_q;

                % Compute forward kinematics for q_plus
                obj.setQ(q_plus)
                g_r_EE_plus = obj.getEndeffectorPos;

                % Compute forward kinematics for q_minus
                obj.setQ(q_minus)
                g_r_EE_minus = obj.getEndeffectorPos;

                % Compute derivative
                J(:, i) = (g_r_EE_plus - g_r_EE_minus) / (2 * obj.delta_q);
            end
            % Restore original q
            obj.setQ(q_save);
        end

        function is_singularity = checkSingularity(obj)
            % Check Singularity
            is_singularity = false;
            if cond(obj.getJacobianNumeric) > obj.singularityThreshold
                is_singularity = true;
            end
        end

        %% Plotting

        function initRobotPlot(obj)
            % Initialize the figure
            close all;
            obj.fig = figure;
            hold on
            view(-290, 25);
            axis equal;
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            grid on;
            xlim([-600, 600]);
            ylim([-600, 600]);
            zlim([0, 650]);
            camva(5.368090431213385);
            screenSize = get(0, 'ScreenSize');
            figWidth = 2 * screenSize(3) / 3;
            set(obj.fig, 'Position', [1, 1, figWidth, screenSize(4)]);

            % Initialize the plot for each link
            for i = 1:length(obj.links)
                obj.links(i).initLinkPlot();
            end

            % Initialize the frame plots of first and last frame
            obj.frames(1).initFramePlot();
            obj.frames(end).initFramePlot();

            % Initialize the trajectory plot
            obj.initTrajectoryPlot();

            % Initialize the workspace plot
            obj.workspace.initWorkspacePlot();

            % Initialize the TCP position text
            obj.initTcpPositionText();
        end

        function updateRobotPlot(obj)
            % Update the plot for each link
            for i = 1:length(obj.links)
                obj.links(i).updateLinkPlot();
            end

            % Update the frame plots
            obj.frames(1).updateFramePlot();
            obj.frames(end).updateFramePlot();

            % Update the trajectory plot
            obj.updateTrajectoryPlot();

            % Update the TCP position text
            tcp_pos = obj.getEndeffectorPos();
            tcp_pos_str = sprintf('End-Effector Position: [%.2f, %.2f, %.2f]', tcp_pos(1), tcp_pos(2), tcp_pos(3));
            set(obj.tcpTextHandle, 'String', tcp_pos_str);

            drawnow limitrate;
        end

        function initTrajectoryPlot(obj)
            % Get the current end-effector position
            currentEEPos = obj.getEndeffectorPos();

            % Initialize trajectoryPoints as a fixed-size array filled with the current end-effector position
            obj.trajectoryPoints = repmat(currentEEPos, 1, obj.trajectoryLength);

            % Reset the trajectory index
            obj.trajectoryIndex = 1;

            % Create the initial trajectory plot
            obj.trajectoryPlot = plot3(obj.trajectoryPoints(1, :), obj.trajectoryPoints(2, :), obj.trajectoryPoints(3, :), 'k-', 'LineWidth', 1);
        end

        function updateTrajectoryPlot(obj)
            % Get the latest end-effector position
            newPoint = obj.getEndeffectorPos();

            % Update the trajectory point at the current index
            obj.trajectoryPoints(:, obj.trajectoryIndex) = newPoint;

            % Increment the index and wrap around if necessary
            obj.trajectoryIndex = mod(obj.trajectoryIndex, obj.trajectoryLength) + 1;

            % Update the trajectory plot
            % Roll the data to align with the current trajectory order
            rolledData = circshift(obj.trajectoryPoints, [0, -obj.trajectoryIndex + 1]);
            set(obj.trajectoryPlot, 'XData', rolledData(1, :), 'YData', rolledData(2, :), 'ZData', rolledData(3, :));
        end

        function initTcpPositionText(obj)
            % Initialize the TCP position text
            % Position the text at a fixed location in the plot
            tcp_pos_str = 'End-Effector Position: [0.00, 0.00, 0.00]';
            obj.tcpTextHandle = text(0, 0, 680, tcp_pos_str, ...
                'FontSize', 10, ...
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'bottom');
        end

        function updateTcpPositionText(obj)
            % Update the TCP position text
            tcp_pos = obj.getEndeffectorPos();
            tcp_pos_str = sprintf('End-Effector Position: [%.2f, %.2f, %.2f]', ...
                tcp_pos(1), tcp_pos(2), tcp_pos(3));
            set(obj.tcpTextHandle, 'String', tcp_pos_str);
        end

    end
end



