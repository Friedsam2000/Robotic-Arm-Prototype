classdef Set_Joints < AbstractProgram

    properties (Constant)
        default_Kp = 2;
        default_precision_deg = 0.5;
    end


    methods

        function start(programObj,varargin)
            
            % Update Config and Plot
            programObj.launcher.updateConfigAndPlot;

            % Parse optional arguments
            p = inputParser;
            addRequired(p, 'q_desired', @(x) isvector(x) && length(x) == 4 && all(isnumeric(x)) && iscolumn(x));
            addParameter(p, 'Kp', programObj.default_Kp);
            addParameter(p, 'precision_deg', programObj.default_precision_deg);
            parse(p, varargin{:});
            q_desired = p.Results.q_desired;
            Kp = p.Results.Kp;
            precision_deg = p.Results.precision_deg;
            precision_rad = deg2rad(precision_deg);

            % Control Loop executes while the program is not deleted or a break
            % condition is met (e.g. position reached)
            while isvalid(programObj)

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
                    fprintf("Program %s: Position Reached within Tolerance \n", class(programObj))
                    break;
                end
            end
            delete(programObj)
        end
    end
end