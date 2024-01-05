classdef FollowTrajectoryProgram < Program
    properties
        trajectoryHeight
        trajectoryTime
    end

    methods
        function obj = FollowTrajectoryProgram(launcherObj, varargin)
            % Call superclass constructor
            obj@Program(launcherObj);

            % Parse optional arguments
            p = inputParser;
            addOptional(p, 'trajectoryHeight', 400); % Default height
            addOptional(p, 'trajectoryTime', 10); % Default time
            parse(p, varargin{:});

            obj.trajectoryHeight = p.Results.trajectoryHeight;
            obj.trajectoryTime = p.Results.trajectoryTime;
        end

        function execute(obj)

            % Setup a cleanup function that gets called when Strg + C
            % during loop
            cleanupObj = onCleanup(@() obj.cleanup());
            obj.launcher.realRobot.torqueEnable;

            % Initial drawing
            q = obj.launcher.realRobot.getQ;
            obj.launcher.virtualRobot.setQ(q);
            obj.launcher.virtualRobot.draw;
            obj.launcher.virtualRobot.workspace.draw;

            % Controller and Planner
            controller = NullspaceController(obj.launcher.virtualRobot);
            planner = PathPlanner2D(obj.launcher.virtualRobot, obj.trajectoryHeight);
            planner.userInputPath; % User inputs path

            % Trajectory Generator
            trajectoryGenerator = TrajectoryGenerator(planner.getWaypointList, obj.trajectoryTime);
            [x_d, v_d, t] = trajectoryGenerator.getTrajectory;

            % Draw the desired trajectory
            obj.launcher.virtualRobot.draw;
            trajectoryGenerator.draw(obj.launcher.virtualRobot.fig);
            obj.launcher.virtualRobot.workspace.draw;

            % Control Loop
            loopBeginTime = tic;
            while true
                % Update virtual robot
                q = obj.launcher.realRobot.getQ;
                obj.launcher.virtualRobot.setQ(q);

                % Check Singularity
                J = obj.launcher.virtualRobot.getJacobianNumeric;
                if cond(J) > 25
                    disp("Program: Close to Singularity. Stopping.")
                    break
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
                disp(q_dot)
                % Update real robot
                obj.launcher.realRobot.setJointVelocities(q_dot);
                obj.launcher.virtualRobot.draw;

                pause(0.01); % Short pause to yield execution
            end

            % Cleanup
            obj.stop();
        end

        function stop(obj)
            % Stop method implementation
            obj.launcher.realRobot.setJointVelocities([0; 0; 0; 0]);
            obj.launcher.notifyProgramStopped(obj);  % Notify Launcher about the stop
        end

        function cleanup(obj)
            % Cleanup actions
            obj.stop();
        end
    end
end
