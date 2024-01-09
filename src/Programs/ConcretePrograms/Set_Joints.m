classdef Set_Joints < AbstractProgram

    properties (Constant)
        default_Kp = 2;
        default_precision_deg = 0.5;
    end


    methods

        function concreteProgram(obj, varargin)

            % Parse optional arguments
            p = inputParser;
            addRequired(p, 'q_desired', @(x) isvector(x) && length(x) == 4 && all(isnumeric(x)) && iscolumn(x));
            addParameter(p, 'Kp', obj.default_Kp);
            addParameter(p, 'precision_deg', obj.default_precision_deg);
            parse(p, varargin{:});
            q_desired = p.Results.q_desired;
            Kp = p.Results.Kp;
            precision_deg = p.Results.precision_deg;
            precision_rad = deg2rad(precision_deg);

            % P-Control Loop for Joint Positions
            while strcmp(obj.launcher.status, 'busy')

                % Update virtual robot and plot
                obj.updateConfigAndPlot;
                q = obj.launcher.virtualRobot.getQ;

                % Calculate remaining errors
                currentError = q_desired - q;

                % Set velocities
                PID_velocities = Kp * currentError;
                obj.launcher.realRobot.setJointVelocities(PID_velocities);

                % Check if joints converged
                if all(abs(currentError) <= precision_rad)
                    fprintf("Program %s: Position Reached within Tolerance \n", class(obj))
                    break;
                end
                

            end
            delete(obj)
        end
    end
end