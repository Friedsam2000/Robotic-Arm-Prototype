classdef Trajectory2D < Program

    properties (Constant)
        % Kp Gain and ramp duration 
        default_Kp = 1;
        ramp_duration = 1; % [s]
    end

    properties
        controller = [];
        Kp = [];
        % Desired trajectory (pos, velocity, time)
        x_d = [];
        v_d = [];
        t = [];
        start_time = [];
    end

    methods

        function setup(obj,varargin)

            % Dont start the program if in singularity
            if obj.launcher.virtualRobot.checkSingularity
                obj.stopCondition = true;
                return;
            end

            % Parse optional arguments
            p = inputParser;
            addRequired(p, 'trajectoryTime', @(x) isscalar(x) && isnumeric(x));
            addOptional(p, 'Kp', obj.default_Kp); % Default Kp value
            parse(p, varargin{:});
            trajectoryTime = p.Results.trajectoryTime;
            obj.Kp = p.Results.Kp; % Final Kp value

            % Initialize the controller
            obj.controller = NullspaceController(obj.launcher.virtualRobot);

            % Initialize the planner
            planner = PathPlanner(obj.launcher.virtualRobot);
            % Let the user draw a path in the X-Y plane at the current height
            currentPosition = obj.launcher.virtualRobot.forwardKinematics;
            planner.userInputPath(currentPosition(3));
            
            % Initialize the trajectory generator
            trajectoryGenerator = TrajectoryGenerator();
            % Generate a spline interpolated trajectory for the given path and duration
            trajectoryGenerator.generateTrajectory(planner.path,trajectoryTime)
            obj.x_d = trajectoryGenerator.desiredPositionsArray;
            obj.v_d = trajectoryGenerator.desiredVelocitiesArray;
            obj.t = trajectoryGenerator.timeArray;

            % Plot the desired trajectory
            trajectoryGenerator.draw(obj.launcher.virtualRobot.fig)

            % Save program start time
            obj.start_time = tic;
        end

        function loop(obj)
            
            % Stop if approaching singularity
            if obj.launcher.virtualRobot.checkSingularity
                obj.stopCondition = true;
                return;
            end

            % Calculate elapsed time
            elapsed_time = toc(obj.start_time);

            % Ramp Kp
            if elapsed_time < obj.ramp_duration
                obj.controller.Kp = obj.Kp * (elapsed_time / obj.ramp_duration);
            else
                obj.controller.Kp = obj.Kp;
            end

            % Calculate which trajectory index (timestep) matches with the
            % elapsed time
            [~, index] = min(abs(obj.t - elapsed_time));
            if index >= length(obj.t)
                fprintf("Program %s: Trajectory time elapsed. \n", class(obj));
                obj.stopCondition = true;
                return;
            end

            % Set velocities
            q_dot = obj.controller.calcJointVelocities(obj.x_d(:, index), obj.v_d(:, index));
            obj.launcher.realRobot.setJointVelocities(q_dot);
        end
    end
end
