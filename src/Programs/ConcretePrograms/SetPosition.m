classdef SetPosition < Program

    properties (Constant)
        % Kp Gain and ramp duration 
        default_Kp = 1;
        ramp_duration = 1; % [s]
        precision = 1; % mm
         % The program stops if the endeffector stays within the desired
         % position (+ tolerance) for breakTimerStopValue seconds
        breakTimerStopValue = 2; % s
    end

    properties
        controller = [];
        Kp = [];
        x_d = [];
        breakTimerStarted = false;
        breakTimerValue = 0;
        start_time = [];
    end

    methods

        function setup(obj, varargin)

            % Dont start the program if in singularity
            if obj.launcher.virtualRobot.checkSingularity
                obj.stopCondition = true;
                delete(obj);
            end

            % Parse input arguments
            p = inputParser;
            addRequired(p, 'x_d', @(x) isvector(x) && length(x) == 3 && all(isnumeric(x)) && iscolumn(x));
            addOptional(p, 'Kp', obj.default_Kp);
            parse(p, varargin{:});
            obj.x_d = p.Results.x_d;
            obj.Kp = p.Results.Kp;

            % Initialize the controller
            obj.controller = NullspaceController(obj.launcher.virtualRobot);

            % Plot desired position
            scatter3(obj.x_d(1), obj.x_d(2), obj.x_d(3), 'm', 'filled');

            % Save program start time
            obj.start_time = tic;
        end

        function loop(obj)

            obj.stopCondition = obj.launcher.virtualRobot.checkSingularity;

            % Calculate elapsed time
            elapsed_time = toc(obj.start_time);

            % Ramp Kp
            if elapsed_time < obj.ramp_duration
                obj.controller.Kp = obj.Kp * (elapsed_time / obj.ramp_duration);
            else
                obj.controller.Kp = obj.Kp;
            end

            % Get the distance to the goal
            distance_to_goal = norm(obj.x_d - obj.launcher.virtualRobot.forwardKinematics);
            fprintf("Program: Distance to Goal: %.1f \n", distance_to_goal);
            if distance_to_goal < obj.precision
                if ~obj.breakTimerStarted
                    % Start the break timer
                    obj.breakTimerValue = tic;
                    obj.breakTimerStarted = true;
                elseif toc(obj.breakTimerValue) > obj.breakTimerStopValue
                    % End the program if within precision to goal for long enough time
                    fprintf("Program %s: Position reached within Tolerance \n", class(obj))
                    obj.stopCondition = true;
                    return;
                end
            else
                 % Reset the break timer if no longer within precision to goal
                obj.breakTimerStarted = false;
            end

            % Set velocities
            q_dot = obj.controller.calcJointVelocities(obj.x_d, 0);
            obj.launcher.realRobot.setJointVelocities(q_dot);
        end
    end
end
