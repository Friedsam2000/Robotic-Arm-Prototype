classdef VirtualRobot < handle

    properties (Constant)

        JOINT_ANGLE_LIMITS = [-pi/4, pi/4;
            -pi/4, pi/4;
            -(14/8)*pi, (14/8)*pi;
            -(5/6)*pi, (5/6)*pi];

         % Define a condition number for the Jacobian above when the
         % configuration is considered a singularity configuraion
         % cond(J) > singularityCondition --> checkSingularity() = true;
        singularityCondition = 25;
        
        % Small change in joint angles for numeric differentiation of
        % forward kinematics for jacobian calculation
        delta_q = 0.001;

    end

    properties

        q = [0;0;0;0]; % Current Joint Angles

        links;
        frames;
        workspace;
        trajectory;

        % Plotting
        fig; % The figure in which everything is plotted

    end

    methods

        %% Constructor
        function obj = VirtualRobot

            %% Setup Frames and Joints of the simulated robot
            orig_frame = Frame([0; 0; 0], [], 'Origin', []);
            joint1 = Frame([0; 0; 94.5127], orig_frame, 'Joint 1', 'y');
            joint2 = Frame([0;0;0], joint1, 'Joint 2', 'x');
            joint3 = Frame([0;0;150.0277], joint2, 'Joint 3', 'z');
            joint4 = Frame([0;0;157.8995], joint3, 'Joint 4', 'x');
            endeffector_frame = Frame([0;0;220.40], joint4, 'Endeffector', []);

            %% Setup Links of the simulated robot
            numLinks = 5; % Number of links
            grayLevels = linspace(0.2, 0.8, numLinks);  % Define a range of grayscale values. Start from 0.2 (dark) to 0.8 (lighter)

            link1 = Link(orig_frame, joint1, repmat(grayLevels(1), 1, 3));  % Link 1 with first grayscale value
            link2 = Link(joint1, joint2, repmat(grayLevels(2), 1, 3));  % Link 2 with second grayscale value
            link3 = Link(joint2, joint3, repmat(grayLevels(3), 1, 3));  % and so on...
            link4 = Link(joint3, joint4, repmat(grayLevels(4), 1, 3));
            link5 = Link(joint4, endeffector_frame, repmat(grayLevels(5), 1, 3));

            obj.links = [link1, link2, link3, link4, link5];
            obj.frames = [orig_frame, joint1, joint2, joint3, joint4, endeffector_frame];

            % Create the workspace object with reference to this virtualRobot
            obj.workspace = Workspace(obj);

            % Create the trajectory object with reference to this virtualRobot
            obj.trajectory = Trajectory();
        end

        %% Calculation

        function setJointAngles(obj, q)

            for i = 1:4
                % Update the rotation matrix of the frames, starting from
                % the first frame after the origin frame
                obj.frames(i+1).updateRotationMatrix(q(i));
                obj.q(i) = q(i);
            end

        end

        function q = getJointAngles(obj)
            q = obj.q;
        end

        function [g_r_EE] = forwardKinematics(obj)
            % Call the global position calculation of the last frame
            g_r_EE = obj.frames(end).getGlobalPosition;
        end

        function J = getJacobian(obj)
            % Computes the Jacobian matrix numerically, relating joint velocities to end-effector velocities.
            % Uses finite differences on forward kinematics by perturbing joint angles with 'delta_q'.
            % Numerical methods may offer speed advantages over symbolic ones, but precision can vary.


            % Initialize Jacobian matrix
            J = zeros(3, 4);

            q_local = obj.getJointAngles;

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
                obj.setJointAngles(q_plus)
                g_r_EE_plus = obj.forwardKinematics;

                % Compute forward kinematics for q_minus
                obj.setJointAngles(q_minus)
                g_r_EE_minus = obj.forwardKinematics;

                % Compute derivative
                J(:, i) = (g_r_EE_plus - g_r_EE_minus) / (2 * obj.delta_q);
            end
            % Restore original q
            obj.setJointAngles(q_save);
        end

        function singularityWarning = checkSingularity(obj)
            % Check Singularity
            singularityWarning = false;
            if cond(obj.getJacobian) > obj.singularityCondition
                singularityWarning = true;
            end
        end

        %% Plotting

        function initRobotPlot(obj)
            % Initialize the figure
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
            camva(5.368); % sets the view angle
            screenSize = get(0, 'ScreenSize'); % fill 2/3 of the screen
            figWidth = 2 * screenSize(3) / 3;
            set(obj.fig, 'Position', [1, 1, figWidth, screenSize(4)]);

            % Initialize the plot for each link
            for i = 1:length(obj.links)
                obj.links(i).initLinkPlot();
            end

            % Initialize the plots of first and last frame
            obj.frames(1).initFramePlot();
            obj.frames(end).initFramePlot();

            % Initialize the trajectory plot
            obj.trajectory.initTrajectoryPlot(obj.forwardKinematics());

            % Initialize the workspace plot
            obj.workspace.initWorkspacePlot();

            ax = gca;
            disableDefaultInteractivity(ax)

            set(obj.fig, 'Renderer', 'OpenGL');

        end

        function updateRobotPlot(obj)
            % Update the plot for each link
            for i = 1:length(obj.links)
                obj.links(i).updateLinkPlot();
            end

            % Update the frame plots of first and last frame
            % obj.frames(1).updateFramePlot();
            obj.frames(end).updateFramePlot();

            % Update the trajectory plot
            obj.trajectory.updateTrajectoryPlot(obj.forwardKinematics);

        end
    end
end



