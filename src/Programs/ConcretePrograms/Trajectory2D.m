classdef Trajectory2D < Program

    properties (Constant)
        default_Kp = 2;
    end

    properties
        controller
        Kp_final
        trajectoryHeight
        ramp_duration
        start_time
        x_d
        v_d
        t

    end

    methods

        function setup(obj,varargin)

            if obj.launcher.virtualRobot.checkSingularity
                delete(obj);
            end

            % Get Current Height
            g_r_EE = obj.launcher.virtualRobot.forwardKinematics;
            currentHeight = g_r_EE(3);

            % Parse optional arguments
            p = inputParser;
            addRequired(p, 'trajectoryTime', @(x) isscalar(x) && isnumeric(x));
            addOptional(p, 'Kp', obj.default_Kp); % Default Kp value
            addOptional(p, 'trajectoryHeight', currentHeight); % Default height is current height
            parse(p, varargin{:});
            trajectoryTime = p.Results.trajectoryTime;
            obj.Kp_final = p.Results.Kp; % Final Kp value
            obj.trajectoryHeight = p.Results.trajectoryHeight;

            % Initialize Kp ramp
            obj.ramp_duration = 1; % Ramp duration in seconds

            % Controller and Planner
            obj.controller = NullspaceController(obj.launcher.virtualRobot);
            planner = PathPlanner2D(obj.launcher.virtualRobot, obj.trajectoryHeight);

            % Trajectory Generator
            trajectoryGenerator = TrajectoryGenerator(planner.path, trajectoryTime);
            obj.x_d = trajectoryGenerator.x_d;
            obj.v_d = trajectoryGenerator.v_d;
            obj.t = trajectoryGenerator.t;


            % Draw the desired trajectory
            trajectoryGenerator.draw(obj.launcher.virtualRobot.fig);

            % Control Loop executes while the program is not deleted, the
            % robot is not in a singularity configuration or a break
            % condition is met (e.g. position reached)
            obj.start_time = tic; % Start timer
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

            [~, index] = min(abs(obj.t - elapsed_time));
            if index >= length(obj.t)
                fprintf("Program %s: Trajectory time elapsed. \n", class(obj));
                obj.stopCondition = true;
            end

            % Set velocities
            q_dot = obj.controller.calcJointVelocity(obj.x_d(:, index), obj.v_d(:, index));
            obj.launcher.realRobot.setJointVelocities(q_dot);
        end
    end
end
