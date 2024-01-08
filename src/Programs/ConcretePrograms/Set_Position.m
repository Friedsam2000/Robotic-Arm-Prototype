classdef Set_Position < AbstractProgram


    properties (Constant)

        default_Kp = 0.5;
        default_weight_preffered_config = 0.5;
        default_precision = 3; % mm
    end


    methods

        function execute(obj, x_desired, varargin)

            % Setup a cleanup function that gets called when Strg + C
            % during loop or program crashes
            cleanupObj = onCleanup(@() obj.stop());
            % Set Status
            obj.launcher.status = 'executing';
            % Initial drawing
            obj.updateConfigAndPlot;

            % Parse optional arguments
            p = inputParser;
            addOptional(p, 'Kp', obj.default_Kp); % Default height is current height
            addOptional(p, 'weight_preffered_config', obj.default_weight_preffered_config);
            addOptional(p, 'precision', obj.default_precision);
            parse(p, varargin{:});
            Kp = p.Results.Kp;
            weight_preffered_config = p.Results.weight_preffered_config;
            precision = p.Results.precision;

            % Controller and Planner
            controller = NullspaceController(obj.launcher.virtualRobot);
            controller.Kp = Kp;
            controller.weight_preffered_config = weight_preffered_config;

            % Plot desired position
            scatter3(x_desired(1),x_desired(2),x_desired(3), 'm', 'filled');

            % Control Loop
            while ~obj.launcher.virtualRobot.checkSingularity

                % Update virtual robot and plot
                obj.updateConfigAndPlot;

                % Set velocities
                q_dot = controller.computeDesiredJointVelocity(x_desired, NaN, 0);
                obj.launcher.realRobot.setJointVelocities(q_dot);

                % Print the distance to the goal
                distance_to_goal = norm(x_desired-obj.launcher.virtualRobot.getEndeffectorPos);

                if distance_to_goal < precision
                    if ~breakTimerStarted
                        % Start the break timer
                        breakTimerValue = tic;
                        breakTimerStarted = true;
                    elseif toc(breakTimerValue) >= 3 % 3 Seconds within precision
                        break;
                    end
                else
                    breakTimerStarted = false;  % Reset the timer if the condition is no longer met
                end

                pause(0.01); % Short pause to yield execution
            end
            obj.stop
        end
    end
end
