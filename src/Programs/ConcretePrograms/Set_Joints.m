classdef Set_Joints < AbstractProgram

    properties (Constant)
        default_Kp = 2;
        default_precision_deg = 0.5;
    end


    methods

        function start(programObj, varargin)
            % Update Config and Plot
            programObj.launcher.updateConfigAndPlot;

            % Parse optional arguments
            p = inputParser;
            addRequired(p, 'q_desired', @(x) isvector(x) && length(x) == 4 && all(isnumeric(x)) && iscolumn(x));
            addParameter(p, 'Kp', programObj.default_Kp);
            addParameter(p, 'precision_deg', programObj.default_precision_deg);
            parse(p, varargin{:});
            q_desired = p.Results.q_desired;
            Kp_final = p.Results.Kp;
            precision_deg = p.Results.precision_deg;
            precision_rad = deg2rad(precision_deg);

            % Initialize Kp ramp
            Kp = 0.1;
            ramp_duration = 1.5; % Ramp duration in seconds
            start_time = tic; % Start timer

            % Control Loop executes while the program is not deleted or a break
            % condition is met (e.g. position reached)
            while 1
                % Calculate elapsed time
                elapsed_time = toc(start_time);

                % Ramp Kp
                if elapsed_time < ramp_duration
                    Kp = Kp_final * (elapsed_time / ramp_duration);
                else
                    Kp = Kp_final;
                end

                % Update virtual robot and plot
                programObj.launcher.updateConfigAndPlot;
                q = programObj.launcher.virtualRobot.getQ;

                % Calculate remaining errors
                currentError = q_desired - q;

                % Set velocities
                PID_velocities = Kp * currentError;
                programObj.launcher.realRobot.setJointVelocities(PID_velocities);

                % Check if joints converged
                if all(abs(currentError) <= precision_rad)
                    fprintf("Program %s: Position reached within Tolerance \n", class(programObj))
                    break;
                end

                % Optional: Add a small pause for loop stability
                pause(0.01);
            end
            delete(programObj)
        end
    end
end