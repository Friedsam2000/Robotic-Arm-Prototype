classdef Trajectory_2D < AbstractProgram

    properties (Constant)

        default_trajectoryTime = 15; % [s]
    end

    methods

        function start(obj,varargin)

            % Setup a cleanup function that gets called when Strg + C
            % during loop or program crashes
            cleanupObj = onCleanup(@() obj.stop());
            % Set Status
            obj.launcher.status = 'executing';
            % Initial drawing
            obj.updateConfigAndPlot;

            % Get Current Height
            g_r_EE = obj.launcher.virtualRobot.getEndeffectorPos;
            currentHeight = g_r_EE(3);

            % Parse optional arguments
            p = inputParser;
            addOptional(p, 'trajectoryHeight', currentHeight); % Default height is current height
            addOptional(p, 'trajectoryTime', obj.default_trajectoryTime);
            parse(p, varargin{:});
            trajectoryHeight = p.Results.trajectoryHeight;
            trajectoryTime = p.Results.trajectoryTime;

            % Controller and Planner
            controller = NullspaceController(obj.launcher.virtualRobot);
            planner = PathPlanner2D(obj.launcher.virtualRobot, trajectoryHeight);
            planner.userInputPath; % User inputs path

            % Trajectory Generator
            trajectoryGenerator = TrajectoryGenerator(planner.getWaypointList, trajectoryTime);
            [x_d, v_d, t] = trajectoryGenerator.getTrajectory;

            % Draw the desired trajectory
            trajectoryGenerator.draw(obj.launcher.virtualRobot.fig);

            % Control Loop
            loopBeginTime = tic;
            % Control Loop
            while ~obj.launcher.virtualRobot.checkSingularity

                % Update virtual robot and plot
                obj.updateConfigAndPlot;

                % Time calculations
                elapsedRealTime = toc(loopBeginTime);
                [~, index] = min(abs(t - elapsedRealTime));
                if index >= length(t)
                    break; % Exit loop at the end of the trajectory
                end

                % Set velocities
                q_dot = controller.computeDesiredJointVelocity(x_d(:, index), NaN, v_d(:, index));
                obj.launcher.realRobot.setJointVelocities(q_dot);

                pause(0.01); % Short pause to yield execution
            end
            obj.stop();
        end
    end
end
