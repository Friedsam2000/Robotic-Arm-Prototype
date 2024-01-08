classdef Trajectory_2D < AbstractProgram

    properties (Constant)
        name = "Trajectory_2D";
    end

    methods

        function execute(obj,varargin)


            % Setup a cleanup function that gets called when Strg + C
            % during loop or program crashes
            cleanupObj = onCleanup(@() obj.cleanup());
            obj.is_running = true;

            % Initial drawing
            obj.updateConfig;

            % Get Current Height
            g_r_EE = obj.launcher.virtualRobot.getEndeffectorPos;
            currentHeight = g_r_EE(3);

            % Parse optional arguments
            p = inputParser;
            addOptional(p, 'trajectoryHeight', currentHeight); % Default height is current height
            addOptional(p, 'trajectoryTime', 10);
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
            while true
                % Update virtual robot
                obj.updateConfig;

                % Check Singularity
                J = obj.launcher.virtualRobot.getJacobianNumeric;
                if cond(J) > 25
                    warning("Program: Close to Singularity. Aborting.");
                    break;
                end

                % Time calculations
                elapsedRealTime = toc(loopBeginTime);
                [~, index] = min(abs(t - elapsedRealTime));
                if index >= length(t)
                    break; % Exit loop at the end of the trajectory
                end

                % Compute velocities
                current_x_d = x_d(:, index);
                current_v_d = v_d(:, index);
                q_dot = controller.computeDesiredJointVelocity(current_x_d, NaN, current_v_d);
                % Update real robot
                obj.launcher.realRobot.setJointVelocities(q_dot);

                pause(0.01); % Short pause to yield execution
            end
            obj.stop();
        end
    end
end
