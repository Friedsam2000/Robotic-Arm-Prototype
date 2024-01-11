classdef Set_Position < AbstractProgram


    properties (Constant)
        default_Kp = 1;
        default_weight_preffered_config = 1;
        default_precision = 5; % mm
    end


    methods

        function start(programObj, varargin)

            % Update Config and Plot
            programObj.launcher.updateConfigAndPlot;

            % Parse input arguments
            p = inputParser;
            addRequired(p, 'x_desired', @(x) isvector(x) && length(x) == 3 && all(isnumeric(x)) && iscolumn(x));
            addOptional(p, 'Kp', programObj.default_Kp); % Default height is current height
            addOptional(p, 'weight_preffered_config', programObj.default_weight_preffered_config);
            addOptional(p, 'precision', programObj.default_precision);
            parse(p, varargin{:});
            x_desired = p.Results.x_desired;
            Kp = p.Results.Kp;
            weight_preffered_config = p.Results.weight_preffered_config;
            precision = p.Results.precision;

            % Controller and Planner
            controller = NullspaceController(programObj.launcher.virtualRobot);
            controller.Kp = Kp;
            controller.weight_preffered_config = weight_preffered_config;

            % Plot desired position
            scatter3(x_desired(1),x_desired(2),x_desired(3), 'm', 'filled');

            % Control Loop
            while ~programObj.launcher.virtualRobot.checkSingularity

                % Set velocities
                q_dot = controller.computeDesiredJointVelocity(x_desired, NaN, 0);
                programObj.launcher.realRobot.setJointVelocities(q_dot);

                % Print the distance to the goal
                distance_to_goal = norm(x_desired-programObj.launcher.virtualRobot.getEndeffectorPos);

                if distance_to_goal < precision
                    if ~breakTimerStarted
                        % Start the break timer
                        breakTimerValue = tic;
                        breakTimerStarted = true;
                    elseif toc(breakTimerValue) >= 3 % 3 Seconds within precision
                        fprintf("Program %s: Position reached within Tolerance \n", class(programObj))
                        break;
                    end
                else
                    breakTimerStarted = false;  % Reset the timer if the condition is no longer met
                end

                % Update virtual robot and plot
                programObj.launcher.updateConfigAndPlot;

            end
            delete(programObj)
        end
    end
end
