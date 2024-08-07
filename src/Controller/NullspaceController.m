classdef NullspaceController < handle

    properties

        % this time is used to predict the maximum change in joint
        % angles in limitJointAngles(). It should be larger then any given
        % control loop time

        dt_max = 0.03; % [s]

    end

    properties

        % PID Gains
        Kp = 1;
        Ki = 0.01;

        % Factor of the of the Null-Space Task velocities
        nullspaceGain = 5; % Staying cloes to the comfort configuration q = [0;0;0;0]

        virtualRobot = [];  % Referenced instance of VirtualRobot

        integralError = [0; 0; 0];
    end


    methods
        function obj = NullspaceController(virtualRobot)
            obj.virtualRobot = virtualRobot;
        end

        function q_dot = calcJointVelocities(obj, desiredPosition, desiredVelocity)

            % Get current end-effector position and virtualRobot configuration once
            q = obj.virtualRobot.getJointAngles;
            currentPosition = obj.virtualRobot.forwardKinematics();

            % Compute desired effective workspace velocity
            v = obj.calcTaskspaceVelocity(desiredPosition, currentPosition, desiredVelocity);

            % Cost function for preffered configuration q = [0;0;0;0]
            % H = 1/2 (q/q_lim - q_desired)^2
            % --> dHdQ = q/(q_lim^2)
            dHdQ = obj.virtualRobot.getJointAngles;
            dHdQ = dHdQ./(obj.virtualRobot.JOINT_ANGLE_LIMITS(:,2).^2);

            % Compute Jacobian and pseudoinverse
            J = obj.virtualRobot.getJacobian();
            pinvJ = pinv(J);
            % Compute Nullspace-Projector
            N = (eye(4) - pinvJ * J);

            % Compute joint-space velocity q_dot
            q_dot = pinvJ * v - obj.nullspaceGain * (N * dHdQ);

            % Ensure compliance with joint angle limits
            q_dot = obj.limitJointAngles(q, q_dot);
        end

        function v = calcTaskspaceVelocity(obj, desiredPosition, currentPosition, desiredVelocity)

            % Calculate taskspaceVelocity (Driftkompensation)

            % Calculate errors
            currentError = desiredPosition - currentPosition; % As they should become 0
            obj.integralError = obj.integralError + currentError;

            % Implement anti-windup by limiting the integral error
            integralErrorMax = 60; % Set this to a suitable maximum value
            obj.integralError = min(obj.integralError, integralErrorMax);
            obj.integralError = max(obj.integralError, -integralErrorMax);

            % Compute effective workspace velocity
            % Compute velocity due to P-Gain
            p_vel = currentError * obj.Kp;

            % Compute velocity due to I-Gain
            i_vel = obj.integralError * obj.Ki;

            % Combine P and I velocity components with desired velocity
            v = p_vel + i_vel + desiredVelocity;



        end

        function q_dot = limitJointAngles(obj, q, q_dot)

            for i = 1:length(q)
                if q(i) < obj.virtualRobot.JOINT_ANGLE_LIMITS(i,1) && q_dot(i) < 0
                    % If joint is in lower limit and keeps moving in this direction, only allow motion away from the limit
                    q_dot(i) = 0;
                    fprintf('Warning: Joint %d is approaching lower limit. Angle: %.2f °.\n', i, rad2deg(q(i)));
                elseif q(i) > obj.virtualRobot.JOINT_ANGLE_LIMITS(i,2) && q_dot(i) > 0
                    % If joint is in upper limit and keeps moving in this direction, only allow motion away from the limit
                    q_dot(i) = 0;
                    fprintf('Warning: Joint %d is approaching upper limit. Angle: %.2f °.\n', i, rad2deg(q(i)));
                end
            end
        end
    end
end
