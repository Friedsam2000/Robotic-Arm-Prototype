classdef Set_Position < AbstractProgram


    properties (Constant)

        name = "Set_Position";

    end


    methods


        function execute(obj, x_desired, varargin)

            % Setup a cleanup function that gets called when Strg + C
            % during loop or program crashes
            cleanupObj = onCleanup(@() obj.cleanup());

            % Initial drawing
            obj.updateConfig;

            % Parse optional arguments
            % p = inputParser;
            % addOptional(p, 'trajectoryHeight', currentHeight); % Default height is current height
            % parse(p, varargin{:});
            % trajectoryHeight = p.Results.trajectoryHeight;

            % Controller and Planner
            controller = NullspaceController(obj.launcher.virtualRobot);

            % Control Loop
            while true
                % Update virtual robot
                obj.updateConfig;
                scatter3(x_desired(1),x_desired(2),x_desired(3), 'm', 'filled');

                % Check Singularity
                J = obj.launcher.virtualRobot.getJacobianNumeric;
                if cond(J) > 25
                    disp("Program: Close to Singularity. Stopping.")
                    break
                end

                q_dot = controller.computeDesiredJointVelocity(x_desired, NaN, 0);

                % Update real robot
                obj.launcher.realRobot.setJointVelocities(q_dot);

                % Print the distance to the goal
                distance_to_goal = norm(x_desired-obj.launcher.virtualRobot.getEndeffectorPos);
                printf('Distance to goal: %.0f mm \n', distance_to_goal);

                if distance_to_goal < 5
                    if ~breakTimerStarted
                        % Start the break timer
                        breakTimerValue = tic;
                        breakTimerStarted = true;
                    elseif toc(breakTimerValue) >= 3
                        break;
                    end
                else
                    breakTimerStarted = false;  % Reset the timer if the condition is no longer met
                end

                pause(0.01); % Short pause to yield execution
            end
            obj.stop();
        end
    end
end
