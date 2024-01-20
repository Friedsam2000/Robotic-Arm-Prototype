classdef SetJoints < Program

    properties (Constant)
        % Kp Gain and Kp Ramp duration
        default_Kp = 2;
        ramp_duration = 1; % [s]
        default_precision_deg = 0.2; % [°]


        % Ki Gain and anti-windup limit
        default_Ki = 0.03; % You need to tune this value
        integralErrorMax = 0.3; % Adjust as needed
    end

    properties
        Kp = [];
        Ki = []; % Integral gain
        precision_rad = []; % [rad]
        q_desired = []; % [rad]
        start_time = [];
        integralError = 0; % To accumulate the error over time
        breakTimerStarted = false;
        breakTimerValue = 0;
        breakTimerStopValue = 0; % Time in seconds to stay within precision

    end

    methods

        function setup(obj, varargin)

            % Parse optional arguments
            p = inputParser;
            addRequired(p, 'q_desired_deg', @(x) isvector(x) && length(x) == 4 && all(isnumeric(x)) && iscolumn(x));
            addParameter(p, 'Kp', obj.default_Kp);
            addParameter(p, 'precision_deg', obj.default_precision_deg);
            addParameter(p, 'Ki', obj.default_Ki);
            addParameter(p, 'breakTimerStopValue', obj.breakTimerStopValue); % New parameter
            parse(p, varargin{:});
            obj.q_desired = deg2rad(p.Results.q_desired_deg);
            obj.Kp = p.Results.Kp;
            obj.precision_rad = deg2rad(p.Results.precision_deg);
            obj.Ki = p.Results.Ki;
            obj.breakTimerStopValue = p.Results.breakTimerStopValue; % New property


            % Save program start time
            obj.start_time = tic;

        end

        function loop(obj)

            % Calculate elapsed time
            elapsed_time = toc(obj.start_time);

            % Ramp Kp
            if elapsed_time < obj.ramp_duration
                K = obj.Kp * (elapsed_time / obj.ramp_duration);
            else
                K = obj.Kp;
            end

            % Calculate remaining errors
            q = obj.launcher.virtualRobot.getJointAngles;
            currentError = obj.q_desired - q;

            % Check if joints converged within precision
            if all(abs(currentError) <= obj.precision_rad)
                if ~obj.breakTimerStarted
                    % Start the break timer
                    obj.breakTimerValue = tic;
                    obj.breakTimerStarted = true;
                elseif toc(obj.breakTimerValue) > obj.breakTimerStopValue
                    % End the program if within precision for the required duration
                    fprintf("Program %s: Position maintained within Tolerance \n", class(obj));
                    obj.stopCondition = true;
                    return;
                end
            else
                % Reset the break timer if no longer within precision
                obj.breakTimerStarted = false;
            end

            % Calculate and clamp the integral error for anti-windup
            obj.integralError = obj.integralError + currentError;
            obj.integralError = min(max(obj.integralError, -obj.integralErrorMax), obj.integralErrorMax);

            disp('Ki-Gain: ')
            disp(norm(obj.Ki * obj.integralError));
            disp('Kp-Gain: ')
            disp(norm(K * currentError));
            disp('')

            % Set velocities (PI-Controller)
            P_velocities = K * currentError;
            I_velocities = obj.Ki * obj.integralError;
            PID_velocities = P_velocities + I_velocities;

            obj.launcher.realRobot.setJointVelocities(PID_velocities);

        end
    end
end