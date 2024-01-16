classdef NullspaceController < handle

    properties

        % PID Gains
        Kp = 1;
        Ki = 0;

        % Factor of the of the Null-Space Task velocities
        nullspaceGain = 5; % Staying cloes to the comfort configuration q = [0;0;0;0]

        virtualRobot  % Referenced instance of VirtualRobot

        integralError;
    end

    
    methods
        function obj = NullspaceController(virtualRobot)
            obj.virtualRobot = virtualRobot;
            obj.integralError = [0; 0; 0];
        end
        
        function q_dot = calcJointVelocity(obj, desiredPosition, desiredVelocity)

            % Get current end-effector position and virtualRobot configuration once
            q = obj.virtualRobot.getJointAngles;
            currentPosition = obj.virtualRobot.forwardKinematics();
            
            % Compute desired effective workspace velocity
            v = obj.calcTaskspaceVelocity(desiredPosition, currentPosition, desiredVelocity);
           
            % Cost function for preffered configuration q = [0;0;0;0]
            % H = 1/2 (q - q_desired)^2 
            % --> dHdQ = q
            dHdQ = obj.virtualRobot.getJointAngles;
            % Normalize with range
            dHdQ = dHdQ./obj.virtualRobot.JOINT_ANGLE_LIMITS(:,2);

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
            currentError = desiredPosition-currentPosition; % As they should become 0
            obj.integralError = obj.integralError + currentError;
           
            % Compute effective workspace velocity
            v = currentError * obj.Kp   + obj.integralError * obj.Ki + desiredVelocity;
            
        end

        function q_dot = limitJointAngles(obj, q, q_dot)
            % Predicted next joint configuration
            q_next = q + q_dot * 0.3; % this timestep should be bigger then the control loop dt
        
            % Check if the predicted next configuration violates the joint limits
            for idx = 1:length(q)
                if q_next(idx) < obj.virtualRobot.JOINT_ANGLE_LIMITS(idx,1) && q_dot(idx) < 0
                    % If joint is moving towards lower limit and is too close, only allow motion away from the limit
                    q_dot(idx) = 0;
                    fprintf('Warning: Joint %d is approaching lower limit. Angle: %.2f °.\n', idx, rad2deg(q(idx)));
                elseif q_next(idx) > obj.virtualRobot.JOINT_ANGLE_LIMITS(idx,2) && q_dot(idx) > 0
                    % If joint is moving towards upper limit and is too close, only allow motion away from the limit
                    q_dot(idx) = 0;
                    fprintf('Warning: Joint %d is approaching upper limit. Angle: %.2f °.\n', idx, rad2deg(q(idx)));
                end
            end
        end
   end
end
