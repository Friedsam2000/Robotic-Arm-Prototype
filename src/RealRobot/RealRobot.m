classdef RealRobot < handle

    properties (Constant)

        % Expected Servo IDs (Can be configured in Dynamixel Wizard)
        % ID 1 : ShoulderServoOne  --> located in x-direction  (front)
        % ID 2 : ShoulderServoTwo
        % ID 3 : YawServo
        % ID 4 : ElbowServo

        JOINT_VELOCITY_LIMITS = [0.6;0.6;2;2];
        SHOULDER_GEAR_RATIO = 5;
        ELBOW_GEAR_RATIO = 2.5;

    end

    properties

        %ServoChain Object
        servoChain = [];

        % The Servo Angles that correspond to Joint Angles q = [0;0;0;0]
        servoZeroAngles = [-inf,-inf,-inf,-inf];

    end


    methods
        %% Constructor = Connection Attempt, Calls destructor if failed
        function obj = RealRobot(dynamixel_lib_path, port)
            obj.servoChain = ServoChain.getInstance(dynamixel_lib_path, port);
        end

        %% Destructor
        function delete(obj)
           
            fprintf("RealRobot: Destructor called.\n")
            delete(obj.servoChain)
            obj.servoChain = [];
        end

        %% Send/Receive 
        function [jointAngles] = getJointAngles(obj)

            %Get the angles of the joints q in RAD

            % joint1 : bevel rotation around fixed y-Axis
            % joint2 : bevel rotation around dependent x-Axis
            % joint3 : yaw rotation around z-Axis
            % joint4 : elbow rotatoin around x-Axis

            % Check if Zero Position has been set
            if isinf(sum(obj.servoZeroAngles))
                error("RealRobot: Could not get Joint Angles. Zero position of the robot is not set.")
            end

            % Get all servo angles phi in RAD
            servoAngles = obj.servoChain.getServoAngles();
            if ~obj.checkConnection()
                return;
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
                if abs(jointVelocities(ID)) > obj.JOINT_VELOCITY_LIMITS(ID)
                    jointVelocities(ID) = obj.JOINT_VELOCITY_LIMITS(ID) * sign(jointVelocities(ID));
                end
            end

            % Convert joint velocities q_dot to servo velocities omega
            servoVelocities = obj.convertJointVelocitiesToServoVelocites(jointVelocities);

            % Ser ServoVelocities in rev/min
            obj.servoChain.setServoVelocities(servoVelocities)
            obj.checkConnection();

        end

        function setRobotTorque(obj, state)
    
            
            % Enable / Disable the torque of the whole robot.
            for ID = 1:4
                obj.servoChain.setServoTorque(ID,state);
                if ~obj.checkConnection()
                    return;
                end
            end
        end

        function state = getRobotTorque(obj)
            % Enable / Disable the torque of the whole robot.
            state = obj.servoChain.getServoTorque(1);
            % Sync Torque of all Servos
            obj.setRobotTorque(state);
        end

        function zeroAtCurrent(obj)
            % Store the current servo angles as zero angles
            obj.servoZeroAngles = obj.servoChain.getServoAngles();
            obj.checkConnection();
        end

        %% Conversio between Joint Angles and Servo Angles (from Thesis Zeitler 2023)

        function jointAngles = convertServoAnglesToJointAngles(obj,servoAngles)

            phi_1_0 = obj.servoZeroAngles(1);
            phi_2_0 = obj.servoZeroAngles(2);
            phi_3_0 = obj.servoZeroAngles(3);
            phi_4_0 = obj.servoZeroAngles(4);

            phi_1 = servoAngles(1);
            phi_2 = servoAngles(2);
            phi_3 = servoAngles(3);
            phi_4 = servoAngles(4);


            % Calculation according to Thesis (corrected)
            delta_phi_1 = phi_1_0 - phi_1;
            delta_phi_2 = phi_2_0 - phi_2;
            q_1 = (delta_phi_1 - delta_phi_2)/obj.SHOULDER_GEAR_RATIO;
            q_2 = -(delta_phi_1 + delta_phi_2)/obj.SHOULDER_GEAR_RATIO;

            q_3 = phi_3 - phi_3_0;
            q_4 = (phi_4 - phi_4_0)/obj.ELBOW_GEAR_RATIO;

            jointAngles = [q_1;q_2;q_3;q_4];

        end

        function servoVelocities = convertJointVelocitiesToServoVelocites(obj,jointVelocities)

            q_1_dot = jointVelocities(1);
            q_2_dot = jointVelocities(2);
            q_3_dot = jointVelocities(3);
            q_4_dot = jointVelocities(4);

            % Calculation according to Thesis
            omega_3 = q_3_dot;
            omega_4 = q_4_dot*obj.ELBOW_GEAR_RATIO;

            omega_1 = 0.5*(q_2_dot-q_1_dot) * obj.SHOULDER_GEAR_RATIO;
            omega_2 = +0.5*(q_2_dot+q_1_dot) * obj.SHOULDER_GEAR_RATIO;

            % Servo velocities are currently in rad/s
            servoVelocities_rad_s = [omega_1;omega_2;omega_3;omega_4];

            % Convert rad/s to rev/min
            servoVelocities = 60*(servoVelocities_rad_s)/(2*pi);

        end
    
    end
        methods
        function state = checkConnection(obj)
            state = true;
            if isempty(obj.servoChain) || ~isvalid(obj.servoChain) || (length(obj.servoChain.availableIDs) ~= 4)
                state = false;
                delete(obj)
                return;
            end
        end
    end
end
