classdef VirtualRobot < handle

    properties (SetAccess = private)
        links   % An array of Link objects, defining the physical connections between joints
        frames % An array of Frame objects, defining joints and frames
        fig % The figure in which everything is visualized
        workspace % A workspace object

        joint_limits = [-pi/4, pi/4;
        -pi/4, pi/4; 
        -(14/8)*pi, (14/8)*pi; 
        -(5/6)*pi, (5/6)*pi];


        % Preallocate memory for trajectory points
        trajectoryPoints = zeros(3, 100); % Preallocate for 100 points

    end

    properties
        trajectoryLength = 1000;  % Default value for trajectory length
    end

    properties (Access = private)

        trajectoryPlot % Handle for the trajectory plot

        q = [0;0;0;0]; % Joint Angles

    end

    methods (Access = public)
        
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
            
            % Small change in joint angles
            delta_q = 1e-6;
            
            % Initialize Jacobian matrix
            J = zeros(3, 4);

            q = obj.getQ;

            %Save q to restore it later
            q_save = q;
            
            % For each joint angle
            for i = 1:4
                % Perturb joint angle i
                q_plus = q;
                q_plus(i) = q_plus(i) + delta_q;
                q_minus = q;
                q_minus(i) = q_minus(i) - delta_q;
                
                % Compute forward kinematics for q_plus
                obj.setQ(q_plus)
                g_r_EE_plus = obj.getEndeffectorPos;
                
                % Compute forward kinematics for q_minus
                obj.setQ(q_minus)
                g_r_EE_minus = obj.getEndeffectorPos;
                
                % Compute derivative
                J(:, i) = (g_r_EE_plus - g_r_EE_minus) / (2 * delta_q);
            end
            % Restore original q
            obj.setQ(q_save);
        end

        function draw(obj, varargin)
            % The display method updates and displays all joints and links
            % Activates hold on, no hold off
            obj.ensureFigureExists();
        
            % Set default values for optional argument to draw frames
            draw_frames = 0;
        
            % Draw links
            for i = 1:length(obj.links)
                obj.links(i).draw
            end
        
            % Get the latest end-effector position
            newPoint = obj.getEndeffectorPos();
        
            % Append the new point to the trajectory
            obj.trajectoryPoints = [obj.trajectoryPoints(:, 2:end), newPoint]; % Shift and append new point
        
            % Ensure the trajectory array always has 100 columns
            if size(obj.trajectoryPoints, 2) < 100
                obj.trajectoryPoints = [zeros(3, 100 - size(obj.trajectoryPoints, 2)), obj.trajectoryPoints];
            end
        
           % Draw the trajectory
            if ~isempty(obj.trajectoryPoints)
                if isempty(obj.trajectoryPlot) || ~isgraphics(obj.trajectoryPlot)
                    obj.trajectoryPlot = plot3(obj.trajectoryPoints(1, :), obj.trajectoryPoints(2, :), obj.trajectoryPoints(3, :), 'k-', 'LineWidth', 1);
                else
                    set(obj.trajectoryPlot, 'XData', obj.trajectoryPoints(1, :), 'YData', obj.trajectoryPoints(2, :), 'ZData', obj.trajectoryPoints(3, :));
                end
            end
        
            % Optionally: draw frames (can be slower)
            if length(varargin) >= 1
                draw_frames = varargin{1};
            end
            if draw_frames
                for i = 1:length(obj.frames)
                    obj.frames(i).draw;
                end
            else
                obj.frames(end).draw;
                obj.frames(1).draw;
            end

            % Draw Workspace
            obj.workspace.draw;

        
            drawnow limitrate;
        end

        function clearTrajectory(obj)
            % Get the current end-effector position
            currentEEPos = obj.getEndeffectorPos();
    
            % Fill the trajectoryPoints with the current end-effector position,
            % repeated trajectoryLength times
            obj.trajectoryPoints = repmat(currentEEPos, 1, obj.trajectoryLength);
        end

    end
    
    methods (Access = public, Hidden)

        function ensureFigureExists(obj)
            if isempty(obj.fig) || ~isvalid(obj.fig)
                obj.fig = figure;

                hold on
                view(-290, 25);
                axis equal;
                xlabel('X');
                ylabel('Y');
                zlabel('Z');
                grid on;
        
                % Set fixed axis limits
                xlim([-600, 600]);
                ylim([-600, 600]);
                zlim([0, 650]);
        
                % Set camera view angle
                camva(5.368090431213385);
        
                % Clear Trajectory
                obj.clearTrajectory;
        
                % Position the figure
                screenSize = get(0, 'ScreenSize');
                figWidth = 2 * screenSize(3) / 3;
                set(obj.fig, 'Position', [1, 1, figWidth, screenSize(4)]);
            end
        end

    end
    
end


    
