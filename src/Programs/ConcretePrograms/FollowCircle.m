classdef FollowCircle < Program
    properties (Constant)
        % Constants for controller configuration
        default_Kp = 1; % Default proportional gain
        ramp_duration = 1; % [s] Duration for ramping up Kp
        precision = 2; % mm [Tolerance for reaching the circle's starting point]
    end

    properties
        % Properties for the controller and trajectory
        radius; % Circle radius
        duration; % Duration of circular motion
        controller; % Controller object
        Kp; % Current Kp value
        % Trajectory information
        x_d; % Desired positions for circular motion
        v_d; % Desired velocities for circular motion
        t; % Time stamps for the circular trajectory
        circleStartPoint; % Starting point of the circular trajectory
        start_time; % Start time of the program
        approaching = true; % Flag indicating if still approaching the circle start
    end

    methods
        function setup(obj, varargin)
            % Input parsing and initialization
            p = inputParser;
            addRequired(p, 'params', @(x) isnumeric(x) && numel(x) == 2);
            addParameter(p, 'Kp', obj.default_Kp);
            parse(p, varargin{1}, varargin{2:end});

            params = p.Results.params;
            obj.radius = params(1);
            obj.duration = params(2); % Circular motion duration
            obj.Kp = p.Results.Kp;

            % Singularity check
            if obj.launcher.virtualRobot.checkSingularity
                obj.stopCondition = true;
                return;
            end

            % Controller initialization
            obj.controller = NullspaceController(obj.launcher.virtualRobot);
            obj.controller.Kp = obj.Kp;

            % Calculate the circle's starting point
            currentPosition = obj.launcher.virtualRobot.forwardKinematics;
            obj.circleStartPoint = currentPosition(1:3) + [obj.radius; 0; 0]; % Adjust as needed

            % Circular trajectory generation
            trajectoryGenerator = TrajectoryGenerator();
            obj.generateCircularTrajectory(trajectoryGenerator, obj.radius, obj.duration);

            % Start time recording
            obj.start_time = tic;
        end

        function loop(obj)

            % Check for singularity at the start of each loop iteration
            if obj.launcher.virtualRobot.checkSingularity
                obj.stopCondition = true;
                return;
            end

            if obj.approaching
                obj.approachCircleStart();
            else
                obj.followCircularPath();
            end
        end

        function approachCircleStart(obj)
            % Approach the circle's starting point
            currentPos = obj.launcher.virtualRobot.forwardKinematics;
            distanceToStart = norm(currentPos - obj.circleStartPoint);

            if distanceToStart <= obj.precision
                obj.approaching = false; % Transition to circular path
                obj.start_time = tic; % Reset the start time for the circular motion
                return;
            end

            % Calculate and set joint velocities to approach the start point
            q_dot = obj.controller.calcJointVelocities(obj.circleStartPoint, zeros(3,1));
            obj.launcher.realRobot.setJointVelocities(q_dot);
        end


        function followCircularPath(obj)
            % Calculate elapsed time since starting the circular path
            elapsed_time = toc(obj.start_time) - obj.ramp_duration; % Adjust if there was a ramp-up time

            % Determine the current index based on elapsed time
            [~, index] = min(abs(obj.t - elapsed_time));
            if index >= length(obj.t)
                fprintf("Program %s: Circular trajectory completed. \n", class(obj));
                obj.stopCondition = true;
                return;
            end

            % Update controller with the current desired positions and velocities
            desired_position = obj.x_d(:, index);
            desired_velocity = obj.v_d(:, index);
            q_dot = obj.controller.calcJointVelocities(desired_position, desired_velocity);
            obj.launcher.realRobot.setJointVelocities(q_dot);
        end

        function generateCircularTrajectory(obj, trajectoryGenerator, radius, duration)
            currentPosition = obj.launcher.virtualRobot.forwardKinematics;
            height = currentPosition(3); % Z-coordinate for the current height

            % Generate circular path points
            numPoints = 100; % Number of points to define the circle
            theta = linspace(0, 2 * pi, numPoints);
            x = radius * cos(theta) + currentPosition(1);
            y = radius * sin(theta) + currentPosition(2);
            z = repmat(height, 1, numPoints); % Maintain constant height

            path = [x; y; z]; % Assemble the path

            % Use TrajectoryGenerator to create the trajectory
            trajectoryGenerator.generateTrajectory(path, duration);

            % Assign generated trajectory data to class properties
            obj.x_d = trajectoryGenerator.desiredPositionsArray;
            obj.v_d = trajectoryGenerator.desiredVelocitiesArray;
            obj.t = trajectoryGenerator.timeArray;

            % Plot the desired trajectory
            trajectoryGenerator.draw(obj.launcher.virtualRobot.fig)
        end
    end

    methods (Static)
        function argsInfo = getArgumentsInfo()
            % Provides information about the required arguments for FollowCircle
            argsInfo.name = 'Radius [mm];    Duration [s]';
            argsInfo.placeholder = 'radius; duration';
            % Updated pattern to allow optional spaces around semicolon
            argsInfo.validationPattern = '^\d+\s*;\s*\d+$'; 
        end
    end

end
