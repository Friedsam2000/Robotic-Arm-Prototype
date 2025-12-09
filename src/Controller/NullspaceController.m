classdef NullspaceController < handle
    % NullspaceController
    %
    % Supports:
    %  - Velocity control in task space (PI) with joint-limit checks (existing).
    %  - Torque-level force control at the end-effector with a nullspace term
    %    that minimizes joint angles (new): calcJointTorquesFromForce().
    %
    % τ = J' * F_des  -  k_ns * N * dH/dq
    % with N = I - J' * inv(J*J' + λ^2 I) * J    (damped projector)

    properties
        % --- general ---
        virtualRobot = [];    % Reference to VirtualRobot
        dt_max = 0.03;        % Used by limitJointAngles()

        % --- velocity (PI) gains (existing) ---
        Kp = 1;
        Ki = 0.01;
        integralError = [0; 0; 0];

        % --- nullspace ---
        nullspaceGain = 5;    % Pull q toward q_des via nullspace
        q_des = [];           % Desired joint "comfort" posture (default zeros)

        % --- numerics/units ---
        lambdaDamp = 1e-6;    % damping for JJ' inversion in projector
        wrenchToTorqueScale = 1e-3; % set 1e-3 if J is in mm; set 1.0 if in meters

        % Optional joint torque limits [N·m], rows [min max]
        jointTorqueLimits = [];  % e.g., [-3 3; -3 3; -1.5 1.5; -1.5 1.5]
    end

    methods
        function obj = NullspaceController(virtualRobot)
            obj.virtualRobot = virtualRobot;
            if isempty(obj.q_des)
                try
                    obj.q_des = zeros(size(virtualRobot.getJointAngles()));
                catch
                    obj.q_des = []; % fallback; will set later once q known
                end
            end
        end

        % ==============================
        % =  TORQUE-LEVEL FORCE CONTROL
        % ==============================
        function tau = calcJointTorquesFromForce(obj, desiredForce)
            % desiredForce: [3x1] (N) for translational forces (matching J rows)
            % Returns joint torques tau [Nx1] in N·m.

            % Current state
            q = obj.virtualRobot.getJointAngles();
            if isempty(obj.q_des), obj.q_des = zeros(size(q)); end

            % Jacobian (assumed 3xN translational rows)
            J = obj.virtualRobot.getJacobian();
            n = length(q);
            I = eye(n);

            % Damped nullspace projector: N = I - J' * inv(JJ' + λ^2 I) * J
            JJt = J * J.';
            N = I - J.' * ((JJt + (obj.lambdaDamp^2) * eye(size(JJt))) \ J);

            % Gradient of the quadratic comfort cost H = 1/2 (q - q_des)' W (q - q_des)
            dHdq = obj.preferenceGradient(q);

            % Task torque from desired end-effector force
            % NOTE: If J is in mm/rad and F is in N, multiply by 1e-3 to get N·m.
            tau_task = J.' * desiredForce * obj.wrenchToTorqueScale;

            % Total torque = task + nullspace
            tau = tau_task - obj.nullspaceGain * (N * dHdq);

            % Optional torque limiting
            if ~isempty(obj.jointTorqueLimits)
                for i = 1:n
                    mn = obj.jointTorqueLimits(i,1);
                    mx = obj.jointTorqueLimits(i,2);
                    if ~isnan(mn), tau(i) = max(tau(i), mn); end
                    if ~isnan(mx), tau(i) = min(tau(i), mx); end
                end
            end
        end

        function dHdq = preferenceGradient(obj, q)
            % Weight by joint limits if available, otherwise identity
            W = [];
            try
                qmax = obj.virtualRobot.JOINT_ANGLE_LIMITS(:,2); % [Nx1] (rad)
                W = diag(1 ./ (qmax.^2));
            catch
                % No limits available
            end
            if isempty(W)
                W = eye(length(q));
            end

            if isempty(obj.q_des)
                obj.q_des = zeros(size(q));
            end
            dHdq = W * (q - obj.q_des);
        end

        % ======================
        % =  VELOCITY (existing)
        % ======================
        function q_dot = calcJointVelocities(obj, desiredPosition, desiredVelocity)
            % Get current pose and q
            q = obj.virtualRobot.getJointAngles();
            currentPosition = obj.virtualRobot.forwardKinematics();

            % Workspace velocity command (PI)
            v = obj.calcTaskspaceVelocity(desiredPosition, currentPosition, desiredVelocity);

            % Preference gradient toward q = 0 with limit scaling
            dHdQ = obj.preferenceGradient(q);

            % Jacobian & pseudoinverse
            J = obj.virtualRobot.getJacobian();
            pinvJ = pinv(J);
            % Nullspace projector for velocity level
            N = (eye(length(q)) - pinvJ * J);

            % Joint velocity command
            q_dot = pinvJ * v - obj.nullspaceGain * (N * dHdQ);

            % Limit against joint angle bounds
            q_dot = obj.limitJointAngles(q, q_dot);
        end

        function v = calcTaskspaceVelocity(obj, desiredPosition, currentPosition, desiredVelocity)
            % Drift compensation PI in task space (existing)
            currentError = desiredPosition - currentPosition;
            obj.integralError = obj.integralError + currentError;

            % Anti-windup
            integralErrorMax = 60;
            obj.integralError = min(obj.integralError, integralErrorMax);
            obj.integralError = max(obj.integralError, -integralErrorMax);

            % PI + feedforward velocity
            p_vel = obj.Kp * currentError;
            i_vel = obj.Ki * obj.integralError;
            v = p_vel + i_vel + desiredVelocity;
        end

        function q_dot = limitJointAngles(obj, q, q_dot)
            for i = 1:length(q)
                try
                    low = obj.virtualRobot.JOINT_ANGLE_LIMITS(i,1);
                    high = obj.virtualRobot.JOINT_ANGLE_LIMITS(i,2);
                catch
                    low = -inf; high = inf;
                end
                if q(i) < low && q_dot(i) < 0
                    q_dot(i) = 0;
                    fprintf('Warning: Joint %d near lower limit. Angle: %.2f °.\n', i, rad2deg(q(i)));
                elseif q(i) > high && q_dot(i) > 0
                    q_dot(i) = 0;
                    fprintf('Warning: Joint %d near upper limit. Angle: %.2f °.\n', i, rad2deg(q(i)));
                end
            end
        end
    end
end