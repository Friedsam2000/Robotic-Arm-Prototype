classdef SetEndeffectorForce < Program

    properties
        K_gain = 5;                % Nullspace gain (minimize joint angles)
        desired_F = [];
        ctrl = [];                 % NullspaceController instance
        dx_prev = [0;0;0];
        t_prev = 0;
        start_pos = [];
    end

    methods
        function setup(obj, varargin)
            p = inputParser;
            addRequired(p, 'F', @(x) isvector(x) && length(x) == 3 && all(isnumeric(x)) && iscolumn(x));
            addParameter(p, 'Kp', 1);
            parse(p, varargin{:});
            obj.desired_F = p.Results.F;

            obj.t_prev = tic;
            obj.start_pos = obj.launcher.virtualRobot.forwardKinematics;

            % Build controller on the virtual robot
            obj.ctrl = NullspaceController(obj.launcher.virtualRobot);
            obj.ctrl.nullspaceGain = obj.K_gain;
            obj.ctrl.wrenchToTorqueScale = 1e-3; % <-- set to 1.0 if J is [m/rad]
            obj.ctrl.lambdaDamp = 1e-6;          % tweak if needed

            % Real robot torque mode
            r = obj.launcher.realRobot;
            r.setRobotTorque(0)
            r.setRobotOperationMode(0)
            r.setRobotTorque(1)
        end

        function loop(obj)
            r = obj.launcher.realRobot;
            v = obj.launcher.virtualRobot;

            % Abort on kinematic singularity
            obj.stopCondition = v.checkSingularity | v.checkJointLimits;

            % calculate desired force to "stay on track"
            cur_pos = v.forwardKinematics;
            dx = cur_pos - obj.start_pos;
            norm_F = norm(obj.desired_F);
            if norm_F
                dx = dx - dot(dx, obj.desired_F) / norm_F^2 * obj.desired_F; % distance to allowed line of movement
            end
            dx_dot = (dx - obj.dx_prev) ./ toc(obj.t_prev);
            obj.t_prev = tic;
            obj.dx_prev = dx;

            kp_vec = 0.2 * [1; 1; 1];
            kd_vec = 0.01 * [1; 1; 1];
            correction_F = - kp_vec .* dx - kd_vec .* dx_dot;

            % Compute joint torques from desired end-effector force
            tau = obj.ctrl.calcJointTorquesFromForce(obj.desired_F + correction_F);

            % Send torques
            r.setJointTorques(tau);

            %fprintf("avg loop frequency: %.2f Hz\n", obj.launcher.getAvgLoopFreq)
        end
    end

    methods (Static)
        function argsInfo = getArgumentsInfo()
            argsInfo.name = 'TCP Forces [N]';
            argsInfo.placeholder = 'Fx; Fy; Fz';
            argsInfo.validationPattern = '^(-?\d+(\.\d+)?\s*;\s*){2}-?\d+(\.\d+)?$';
        end
    end
end