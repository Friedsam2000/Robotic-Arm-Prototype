classdef SetJoints < Program

    properties (Constant)
        default_Kp = 2;
        default_precision_deg = 0.5;
    end

    properties
        Kp_final
        precision_rad
        q_desired
        ramp_duration
        start_time
    end

    methods

        function setup(obj, varargin)

            % Parse optional arguments
            p = inputParser;
            addRequired(p, 'q_desired_deg', @(x) isvector(x) && length(x) == 4 && all(isnumeric(x)) && iscolumn(x));
            addParameter(p, 'Kp', obj.default_Kp);
            addParameter(p, 'precision_deg', obj.default_precision_deg);
            parse(p, varargin{:});
            q_desired_deg = p.Results.q_desired_deg;
            obj.Kp_final = p.Results.Kp;
            precision_deg = p.Results.precision_deg;
            obj.precision_rad = deg2rad(precision_deg);
            obj.q_desired = deg2rad(q_desired_deg);

            % Initialize Kp ramp
            obj.ramp_duration = 1; % Ramp duration in seconds
            obj.start_time = tic; % Start timer

        end

        function loop(obj)

            % Calculate elapsed time
            elapsed_time = toc(obj.start_time);

            % Ramp Kp
            if elapsed_time < obj.ramp_duration
                Kp = obj.Kp_final * (elapsed_time / obj.ramp_duration);
            else
                Kp = obj.Kp_final;
            end


            % Calculate remaining errors
            q = obj.launcher.virtualRobot.getJointAngles;
            currentError = obj.q_desired - q;

            % Set velocities
            PID_velocities = Kp * currentError;
            obj.launcher.realRobot.setJointVelocities(PID_velocities);

            % Check if joints converged
            if all(abs(currentError) <= obj.precision_rad)
                fprintf("Program %s: Position reached within Tolerance \n", class(obj))
                obj.stopCondition = true;
            end
        end
    end
end