classdef RealRobot < handle

    properties (Constant)

        % Assume following ID's (can be configured in Dynamixel Wizard)
        % ID 1 : ShoulderServoOne 
        % ID 2 : ShoulderServoTwo
        % ID 3 : YawServo
        % ID 4 : ElbowServo

        % [0] ShoulderServoOne --> located in x-direction  (front)
        % [1] ShoulderServoTwo (back)
        % [2] YawServo
        % [3] ElbowServo
        
        % Conifgure maximum absolut joint velocities % RAD/s
        % limit joint 3 to 6.5 rev/min to maintain stability of the internal
        % velocity controller. The controller uses high gains to overcome
        % stick-slip behaviour due to high plain bearing friction
        % x / 60 --> rev / s
        % (x / 60 ) * 2 * pi --> rad /s
        % Servo revs are joint revs 
        q_dot_max = [0.6;0.6;2;2];

        %Transmission ratios
        i_shoulder = 5;
        i_elbow = 2.5;


    end

    properties
        
        % The Servo Angles that are considered as Zero Configuration
        ServoZeroPositions = [-inf,-inf,-inf,-inf];

        %ServoChain Object
        servoChain;

        torqueEnabled;
    end
    

    methods
        function obj = RealRobot(dynamixel_lib_path, PORT)
            obj.servoChain = [];
            obj.servoChain = ServoChain(dynamixel_lib_path, PORT);
            if ~isvalid(obj.servoChain)
                delete(obj)
            end
        end

        function torqueDisable(obj)
            % Enable / Disable the torque of the whole robot.
            for ID = 1:4
                obj.servoChain.torqueEnableDisable(ID,0);
            end
            obj.torqueEnabled = false;
        end

        function torqueEnable(obj)
            % Enable / Disable the torque of the whole robot.

            for ID = 1:4
                obj.servoChain.torqueEnableDisable(ID,1);
            end
            obj.torqueEnabled = true;
        end

        function setZeroPositionToCurrentPosition(obj)
            % Store the current servo angles as zero angles
            for ID = 1:4
                obj.ServoZeroPositions(ID) = obj.servoChain.getServoAngle(ID);
            end
        end
       
        function [jointAngles] = getQ(obj)
            %Get the angles of the joints q in RAD

            % joint1 : bevel rotation around fixed y-Axis
            % joint2 : bevel rotation around dependent x-Axis
            % joint3 : yaw rotation around z-Axis
            % joint4 : elbow rotatoin around x-Axis

            % Check if Zero Position has been set
            if isinf(sum(obj.ServoZeroPositions))
                error("RealRobot: Could not get Joint Angles. Zero position of the robot is not set.")
            end

            % Get all servo angles phi in RAD
            servoAngles = [-inf; -inf; -inf; -inf];
            for ID = 1:4
                servoAngles(ID) = obj.servoChain.getServoAngle(ID);
            end

            %Convert servoAngles phi to jointAngles q
            jointAngles = obj.convertServoAnglesToJointAngles(servoAngles);            

        end

        function setJointVelocities(obj,jointVelocities)
            % Set the angular velocities q_dot of all joints in rad/s.

            % joint1 : bevel rotation around fixed y-Axis
            % joint2 : bevel rotation around dependent x-Axis
            % joint3 : yaw rotation around z-Axis
            % joint4 : elbow rotatoin around x-Ax

            % Ensure velocities do not exceed the configued maximum joint speed
            for ID = 1:4
                if abs(jointVelocities(ID)) > obj.q_dot_max(ID)
                    jointVelocities(ID) = obj.q_dot_max(ID) * sign(jointVelocities(ID));
                    fprintf("Joint %d velocity limited by RealRobot\n", ID)
                end
            end

            % Convert joint velocities q_dot to servo velocities omega
            servoVelocity = obj.convertJointVelocitiesToServoVelocites(jointVelocities); 

            for ID = 1:4
                % Ser ServoVelocities in rev/min
                obj.servoChain.setServoVelocity(ID, servoVelocity(ID))
            end

        end
      
        function jointAngles = convertServoAnglesToJointAngles(obj,servoAngles)
            
            phi_1_0 = obj.ServoZeroPositions(1);
            phi_2_0 = obj.ServoZeroPositions(2);
            phi_3_0 = obj.ServoZeroPositions(3);
            phi_4_0 = obj.ServoZeroPositions(4);

            phi_1 = servoAngles(1);
            phi_2 = servoAngles(2);
            phi_3 = servoAngles(3);
            phi_4 = servoAngles(4);


            % Calculation according to Thesis (corrected)
            delta_phi_1 = phi_1_0 - phi_1;
            delta_phi_2 = phi_2_0 - phi_2;
            q_1 = (delta_phi_1 - delta_phi_2)/obj.i_shoulder;
            q_2 = -(delta_phi_1 + delta_phi_2)/obj.i_shoulder;

            q_3 = phi_3 - phi_3_0;
            q_4 = (phi_4 - phi_4_0)/obj.i_elbow;
            
            jointAngles = [q_1;q_2;q_3;q_4];

        end

        function servoVelocities = convertJointVelocitiesToServoVelocites(obj,jointVelocities)
            
            q_1_dot = jointVelocities(1);
            q_2_dot = jointVelocities(2);
            q_3_dot = jointVelocities(3);
            q_4_dot = jointVelocities(4);
           
            % Calculation according to Thesis (corrected)
            omega_3 = q_3_dot;
            omega_4 = q_4_dot*obj.i_elbow;

            omega_1 = 0.5*(q_2_dot-q_1_dot) * obj.i_shoulder;
            omega_2 = +0.5*(q_2_dot+q_1_dot) * obj.i_shoulder;

            % Servo velocities in rad_s
            servoVelocities_rad_s = [omega_1;omega_2;omega_3;omega_4];

            % Convert rad/s to rev/min
            servoVelocities = 60*(servoVelocities_rad_s)/(2*pi);

        end
    end
end
