classdef GoToPositionProgram < Program


    properties (Constant)

        name = "Set Position";

    end


    properties
        x_desired
    end

    methods
        function obj = GoToPositionProgram(launcherObj,x_desired, varargin)
            % Call superclass constructor
            obj@Program(launcherObj);

            % Parse optional arguments
            obj.x_desired = x_desired;

        end

        function execute(obj)

            % Setup a cleanup function that gets called when Strg + C
            % during loop
            cleanupObj = onCleanup(@() obj.cleanup());
            obj.launcher.realRobot.torqueEnable;

            % Initial drawing
            q = obj.updateConfig;
            
            % Controller and Planner
            controller = NullspaceController(obj.launcher.virtualRobot);

            % Control Loop
            while true
                % Update virtual robot
                q = obj.updateConfig;
                scatter3(obj.x_desired(1),obj.x_desired(2),obj.x_desired(3), 'm', 'filled');

                % Check Singularity
                J = obj.launcher.virtualRobot.getJacobianNumeric;
                if cond(J) > 25
                    disp("Program: Close to Singularity. Stopping.")
                    break
                end

                q_dot = controller.computeDesiredJointVelocity(obj.x_desired, NaN, 0);

                % Update real robot
                obj.launcher.realRobot.setJointVelocities(q_dot);

                % Print the distance to the goal
                distance_to_goal = norm(obj.x_desired-obj.launcher.virtualRobot.getEndeffectorPos);
                fprintf('Distance to goal: %.0f mm \n', distance_to_goal);

                if distance_to_goal < 5
                    if ~breakTimerStarted
                        % Start the break timer
                        breakTimerValue = tic;
                        breakTimerStarted = true;
                    elseif toc(breakTimerValue) >= 3
                        % 5 seconds have passed since the timer started
                        obj.launcher.realRobot.setJointVelocities([0; 0; 0; 0]);
                        break;
                    end
                else
                    breakTimerStarted = false;  % Reset the timer if the condition is no longer met
                end

                pause(0.01); % Short pause to yield execution
            end

            % Cleanup
            obj.stop();
        end
    end
end
