classdef SetPosition < Program


    properties (Constant)
        default_Kp = 1;
        default_precision = 5; % mm
    end

    properties
        x_desired
        Kp_final
        precision
        ramp_duration
        start_time
        controller
        breakTimerStarted
        breakTimerValue
    end

    methods

        function setup(obj, varargin)

            if obj.launcher.virtualRobot.checkSingularity
                delete(obj);
            end

            % Parse input arguments
            p = inputParser;
            addRequired(p, 'x_desired', @(x) isvector(x) && length(x) == 3 && all(isnumeric(x)) && iscolumn(x));
            addOptional(p, 'Kp', obj.default_Kp); % Default Kp value
            addOptional(p, 'precision', obj.default_precision);
            parse(p, varargin{:});
            obj.x_desired = p.Results.x_desired;
            obj.Kp_final = p.Results.Kp; % Final Kp value
            obj.precision = p.Results.precision;

            % Initialize Kp ramp
            obj.ramp_duration = 1; % Ramp duration in seconds
            obj.start_time = tic; % Start timer

            % Controller
            obj.controller = NullspaceController(obj.launcher.virtualRobot);

            % Plot desired position
            scatter3(obj.x_desired(1), obj.x_desired(2), obj.x_desired(3), 'm', 'filled');

            % Initialize break timer flag
            obj.breakTimerStarted = false;
        end

        function loop(obj)

            obj.stopCondition = obj.launcher.virtualRobot.checkSingularity;

            % Calculate elapsed time
            elapsed_time = toc(obj.start_time);

            % Ramp Kp
            if elapsed_time < obj.ramp_duration
                Kp = obj.Kp_final * (elapsed_time / obj.ramp_duration);
            else
                Kp = obj.Kp_final;
            end

            % Update controller Kp
            obj.controller.Kp = Kp;

            % Set velocities
            q_dot = obj.controller.calcJointVelocity(obj.x_desired, 0);
            obj.launcher.realRobot.setJointVelocities(q_dot);

            % Print the distance to the goal
            distance_to_goal = norm(obj.x_desired - obj.launcher.virtualRobot.forwardKinematics);
            if distance_to_goal < obj.precision
                if ~obj.breakTimerStarted
                    % Start the break timer
                    obj.breakTimerValue = tic;
                    obj.breakTimerStarted = true;
                elseif toc(obj.breakTimerValue) >= 2 %  Seconds within precision
                    fprintf("Program %s: Position reached within Tolerance \n", class(obj))
                    obj.stopCondition = true;
                end
            else
                obj.breakTimerStarted = false; % Reset the timer if the condition is no longer met
            end
        end
    end
end
