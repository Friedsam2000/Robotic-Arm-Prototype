classdef SetPosition < Program

    properties (Constant)
        % Kp Gain and ramp duration
        default_Kp = 1.5;
        precision = 3; % mm
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
            obj.controller.Kp = obj.Kp;

            % Plot desired position
            scatter3(obj.x_d(1), obj.x_d(2), obj.x_d(3), 'm', 'filled');
        end

        function loop(obj)

            obj.stopCondition = obj.launcher.virtualRobot.checkSingularity;

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

    methods (Static)
        function argsInfo = getArgumentsInfo()
            argsInfo.name = 'TCP Coordinates [mm]';
            argsInfo.placeholder = 'x; y; z';
            % Pattern to match three doubles with semicolons as separators, allowing negative values and optional spaces around separators
            argsInfo.validationPattern = '^(-?\d+(\.\d+)?\s*;\s*){2}-?\d+(\.\d+)?$';
        end
    end
end
