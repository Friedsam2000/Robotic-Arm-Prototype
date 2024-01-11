classdef Set_Position < AbstractProgram


    properties (Constant)
        default_Kp = 1;
        default_weight_preffered_config = 1;
        default_precision = 5; % mm
    end


    methods

        function concreteProgram(obj, varargin)

            % Parse input arguments
            p = inputParser;
            addRequired(p, 'x_desired', @(x) isvector(x) && length(x) == 3 && all(isnumeric(x)) && iscolumn(x));
            addOptional(p, 'Kp', obj.default_Kp); % Default height is current height
            addOptional(p, 'weight_preffered_config', obj.default_weight_preffered_config);
            addOptional(p, 'precision', obj.default_precision);
            parse(p, varargin{:});
            x_desired = p.Results.x_desired;
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
            while ~obj.launcher.virtualRobot.checkSingularity && strcmp(obj.launcher.status, 'busy')

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
                        fprintf("Program %s: Position reached within Tolerance \n", class(obj))
                        break;
                    end
                else
                    breakTimerStarted = false;  % Reset the timer if the condition is no longer met
                end

            end
            delete(obj)
        end
    end
end
