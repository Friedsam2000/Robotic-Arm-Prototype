classdef Set_Position < AbstractProgram


    properties (Constant)
        default_Kp = 2;
        default_precision = 5; % mm
    end


    methods

        function start(programObj, varargin)
            % Update Config and Plot
            programObj.launcher.updateConfigAndPlot;

            % Parse input arguments
            p = inputParser;
            addRequired(p, 'x_desired', @(x) isvector(x) && length(x) == 3 && all(isnumeric(x)) && iscolumn(x));
            addOptional(p, 'Kp', programObj.default_Kp); % Default Kp value
            addOptional(p, 'precision', programObj.default_precision);
            parse(p, varargin{:});
            x_desired = p.Results.x_desired;
            Kp_final = p.Results.Kp; % Final Kp value
            precision = p.Results.precision;

            % Initialize Kp ramp
            Kp = 0;
            ramp_duration = 1; % Ramp duration in seconds
            start_time = tic; % Start timer

            % Controller
            controller = NullspaceController(programObj.launcher.virtualRobot);

            % Plot desired position
            scatter3(x_desired(1), x_desired(2), x_desired(3), 'm', 'filled');

            % Initialize break timer flag
            breakTimerStarted = false;

            % Control Loop
            while ~programObj.launcher.virtualRobot.checkSingularity
                % Calculate elapsed time
                elapsed_time = toc(start_time);

                % Ramp Kp
                if elapsed_time < ramp_duration
                    Kp = Kp_final * (elapsed_time / ramp_duration);
                else
                    Kp = Kp_final;
                end

                % Update controller Kp
                controller.Kp = Kp;

                % Set velocities
                q_dot = controller.computeDesiredJointVelocity(x_desired, NaN, 0);
                programObj.launcher.realRobot.setJointVelocities(q_dot);

                % Print the distance to the goal
                distance_to_goal = norm(x_desired - programObj.launcher.virtualRobot.forwardKinematics);

                if distance_to_goal < precision
                    if ~breakTimerStarted
                        % Start the break timer
                        breakTimerValue = tic;
                        breakTimerStarted = true;
                    elseif toc(breakTimerValue) >= 2 %  Seconds within precision
                        fprintf("Program %s: Position reached within Tolerance \n", class(programObj))
                        break;
                    end
                else
                    breakTimerStarted = false; % Reset the timer if the condition is no longer met
                end

                % Update virtual robot and plot
                programObj.launcher.updateConfigAndPlot;

            end
            delete(programObj)
        end
    end
end
