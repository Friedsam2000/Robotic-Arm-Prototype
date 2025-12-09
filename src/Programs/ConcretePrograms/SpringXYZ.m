classdef SpringXYZ < Program

    properties
        K_gain = 5;
        goal_pos = [];
        dx_prev = [0;0;0];
        t_prev = 0;
        ctrl = [];
    end

    methods

        function setup(obj, varargin)

            p = inputParser;
            addRequired(p, 'spring');
            addParameter(p, 'Kp', 1);
            parse(p, varargin{:});
            %obj.K_gain = p.Results.Kp;
            obj.goal_pos = obj.launcher.virtualRobot.forwardKinematics;

            obj.t_prev = tic;

            r = obj.launcher.realRobot;
            r.setRobotTorque(0)
            r.setRobotOperationMode(0)
            r.setRobotTorque(1)

             % Build controller on the virtual robot
            obj.ctrl = NullspaceController(obj.launcher.virtualRobot);
            obj.ctrl.nullspaceGain = obj.K_gain;
            obj.ctrl.wrenchToTorqueScale = 1e-3; % <-- set to 1.0 if J is [m/rad]
            obj.ctrl.lambdaDamp = 1e-6;          % tweak if needed
        end

        function loop(obj)

            obj.stopCondition = obj.launcher.virtualRobot.checkSingularity;

            r = obj.launcher.realRobot;
            v = obj.launcher.virtualRobot;

            % calculate desired force
            cur_pos = v.forwardKinematics;
            dx = cur_pos - obj.goal_pos;
            dx_dot = (dx - obj.dx_prev) ./ toc(obj.t_prev);
            obj.t_prev = tic;
            obj.dx_prev = dx;

            kp_vec = 0.2 * [1; 1; 1];
            kd_vec = 0.01 * [1; 1; 1];
            desired_F = - kp_vec .* dx - kd_vec .* dx_dot;

            % apply desired force
            desired_t = obj.ctrl.calcJointTorquesFromForce(desired_F);
            r.setJointTorques(desired_t)

            %fprintf("avg loop frequency: %.2f Hz\n", obj.launcher.getAvgLoopFreq)
        end
    end

    methods (Static)
        function argsInfo = getArgumentsInfo()
            argsInfo.name = '';
            argsInfo.placeholder = '';
            % Pattern to match three doubles with semicolons as separators, allowing negative values and optional spaces around separators
            argsInfo.validationPattern = '.*';
        end
    end
end