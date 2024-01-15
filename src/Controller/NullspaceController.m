classdef NullspaceController < handle

    properties

        Kp = 2;
        Ki = 0;

        % Weights of the Null-Space Tasks (only one can be Non-Zero)
        weight_z = 0; % Weight of the preferred z-Axis Vector
        weight_preffered_config = 2; % Weight of the preferred joint configuration
    end


    properties (Access=private)


        % Variable used for numeric derivation of Cost Function H
        delta_matrix =  0.001 * eye(4);

        virtualRobot  % Referenced instance of VirtualRobot

        % Joint Limits set by the referenced virtualRobot object
        q_max;
        q_min;

        lastPrintTime = 0;    % Timestamp of the last printed warning message
        printInterval = 0.3;    % Minimum time interval between prints in seconds

        integralError = [0; 0; 0];

    end

    
    methods
        function obj = NullspaceController(virtualRobot)
            obj.virtualRobot = virtualRobot;
            obj.q_max = virtualRobot.joint_limits(:,2);
            obj.q_min = virtualRobot.joint_limits(:,1);
            obj.lastPrintTime = tic;
            obj.integralError = [0; 0; 0];


        end
        
        function q_dot = computeDesiredJointVelocity(obj, x_desired, z_desired, v_desired)

            % Get current end-effector position and virtualRobot configuration once
            q = obj.virtualRobot.getJointAngles;
            x_current = obj.virtualRobot.forwardKinematics;
            
            % Compute desired effective workspace velocity
            v_d_eff = obj.computeEffectiveVelocity(x_desired, x_current, v_desired);
           
            % If a desired z-Axis is provided, then the preffered config is
            % deactivated and vise versa
            if obj.weight_z == 0
                dHdQ_preferred_config = obj.numericDiff(@obj.H_prefered_joint_configuration, q);
                dHdQ = obj.weight_preffered_config*dHdQ_preferred_config;
            elseif obj.weight_preffered_config == 0
                dHdQ_z = obj.numericDiff(@obj.H_z_desired, q, z_desired);
                dHdQ = obj.weight_z * dHdQ_z;
            else
                error("Controller Error: Either weight_z or weight_preffered_config must be 0")
            end

            % Compute Jacobian and pseudoinverse
            J = obj.virtualRobot.getJacobian();;
            pinvJ = pinv(J);

            q_dot = obj.computeQdotPseudoinverse(J, pinvJ, v_d_eff, dHdQ);
            
            % Ensure compliance with joint angle limits
            q_dot = obj.ensureJointLimitCompliance(q, q_dot);
        end
    end

    methods (Access=private)

        % Pseudoinverse, Nullspaceprojector method for computing the desired
        % joint velocites q_dot
        function q_dot = computeQdotPseudoinverse(obj, J, pinvJ, u, dHdQ)

            N = (eye(4) - pinvJ * J);
            % Compute joint-space velocity q_dot
            q_dot = pinvJ * u - N * dHdQ';

            % Printing for analyzing cost function impact
            % q_dot_pinv_norm = norm(pinvJ * u);
            % q_dot_cost_norm = norm(N * dHdQ');



        end
        
        function v_d_eff = computeEffectiveVelocity(obj, x_desired, x_current, v_desired)
            % Calculate v_d_eff (Driftkompensation)
            % PI - Controller, could be expanded to PID
            
            % Calculate remaining errors
            currentError = x_desired-x_current; % As they should become 0
            obj.integralError = obj.integralError + currentError;
           
            
            % Compute effective workspace velocity
            v_d_eff = currentError * obj.Kp   + obj.integralError * obj.Ki + v_desired;
            
        end
        
        function H = H_z_desired(obj, z_desired)

            % Get the current z-Axis vectors of the endeffector frame in global
            % frame
            g_A_endeffector = obj.virtualRobot.frames(end).getGlobalRotationMatrix;
            z_endeffector = g_A_endeffector * [0;0;1];

            % Normalize the z-Axis vectors 
            z_desired_normalized = z_desired/norm(z_desired);
            z_endeffector_normalized = z_endeffector/norm(z_endeffector);

            % H = 1/2 * (z(q) - z_desired)^2 
            H = 0.5 * norm(z_endeffector_normalized - z_desired_normalized)^2;

            % fprintf("H-z-desired: %.2f\n", H);

        end

        function H = H_prefered_joint_configuration(obj,q)
            
            % Normalize the joint angles with their max range
            q = q./obj.q_max;
                
            % H = 1/2 (q - q_desired)^2 
            % q_desired = 0 is the comfort position
            
            H = 0.5 * (q' * q);

            % fprintf("H-preferred-config: %.2f\n", H);
        end

        function dHdQ = numericDiff(obj, H_method, q, varargin)
            num_joints = length(q);
            
            % Extract delta_q from the delta_matrix
            delta_q = obj.delta_matrix(1,1); % because it's diagonal and all values are the same
            
            % Perturb joint configurations using broadcasting
            q_plus = q + obj.delta_matrix;
            q_minus = q - obj.delta_matrix;
            
            % Modify the H function call depending on varargin
            if isempty(varargin)
                H_modified = @(q) H_method(q);
            else
                H_modified = @(q) H_method(q, varargin{:});
            end

            % Evaluate H for all perturbed configurations
            H_plus = arrayfun(@(col) H_modified(q_plus(:, col)), 1:num_joints);
            H_minus = arrayfun(@(col) H_modified(q_minus(:, col)), 1:num_joints);
            
            % Compute gradient
            dHdQ = (H_plus - H_minus) / (2 * delta_q);
        end

        function q_dot = ensureJointLimitCompliance(obj, q, q_dot)
            % Predicted next joint configuration
            q_next = q + q_dot * 0.3; % this timestep should be bigger then the control loop dt
        
            % Check if the predicted next configuration violates the joint limits
            for idx = 1:length(q)
                if q_next(idx) < obj.q_min(idx) && q_dot(idx) < 0
                    % If joint is moving towards lower limit and is too close, only allow motion away from the limit
                    q_dot(idx) = 0;
                    fprintf('Warning: Joint %d is approaching lower limit. Angle: %.2f °.\n', idx, rad2deg(q(idx)));
                elseif q_next(idx) > obj.q_max(idx) && q_dot(idx) > 0
                    % If joint is moving towards upper limit and is too close, only allow motion away from the limit
                    q_dot(idx) = 0;
                    fprintf('Warning: Joint %d is approaching upper limit. Angle: %.2f °.\n', idx, rad2deg(q(idx)));
                end
            end
        end
   end
end
