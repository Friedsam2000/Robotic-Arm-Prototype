classdef SetJoints < Program

    properties (Constant)
        % Kp Gain and Kp Ramp duration
        default_Kp = 2;
        default_precision_deg = 0.4; % [°]


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


            % Ensure desired joint angles are within limits
            joint_limits = obj.launcher.virtualRobot.JOINT_ANGLE_LIMITS;
            obj.q_desired = min(max(obj.q_desired, joint_limits(:,1)), joint_limits(:,2));


        end

        function loop(obj)

        
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

            % Set velocities (PI-Controller)
            P_velocities = obj.Kp * currentError;
            I_velocities = obj.Ki * obj.integralError;
            PID_velocities = P_velocities + I_velocities;

            obj.launcher.realRobot.setJointVelocities(PID_velocities);

        end
    end

    methods (Static)
        function argsInfo = getArgumentsInfo()
            argsInfo.name = 'Joint Angles [deg]';
            argsInfo.placeholder = 'q1; q2; q3; q4';
            argsInfo.validationPattern = '^(-?\d+(\.\d+)?[;] *){3}-?\d+(\.\d+)?$'; % Pattern for validation
        end
    end

end