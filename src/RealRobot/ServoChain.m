classdef ServoChain < handle

    properties (Constant)
        % Dynamixel XH-430-W210-T
        PROTOCOL_VERSION            = 2;
        ADDR_PRO_PRESENT_POSITION   = 132;
        ADDR_PRO_GOAL_VELOCITY      = 104;
        BAUDRATE                    = 2000000;
        ADDR_PRO_TORQUE_ENABLE      = 64;
        COMM_SUCCESS                = 0;
        ADDR_PRO_VELOCITY_I_GAIN    = 76;
        ADDR_PRO_VELOCITY_P_GAIN    = 78;
        ADDR_PRO_OPERATION_MODE     = 11;

        ADDR_PRO_PRESENT_CURRENT    = 126;
        ADDR_PRO_GOAL_CURRENT       = 102;
        ADDR_PRO_LED                = 65;

        ADDR_PRO_DRIVE_MODE             = 10;  % Address for the Drive Mode
        TIME_BASED_PROFILE              = 4;   % Value to set for Time-Based Profile (Bit 2 set)
        ADDR_PRO_PROFILE_ACCELERATION   = 108; % Address for Profile Acceleration

    end

    properties
        % IDs of available Dynamixel Servos
        availableIDs = [];

        lib_name = [];
        port_num = [];

        % Group number for sync operations
        groupwrite_num = [];
        groupwrite_cur = [];
        groupread_num = [];
        groupread_cur = [];
    end

    methods (Static)
        % Static method to get the instance of the class
        function single_instance = getInstance(dynamixel_lib_path, port)
            persistent instance;
            if isempty(instance) || ~isvalid(instance)
                instance = ServoChain(dynamixel_lib_path, port);
            end

            single_instance = instance;
        end
    end

    methods ( Access = private)

        %% Constructor = Connection Attempt, Calls destructor if failed
        function obj = ServoChain(dynamixel_lib_path, port)

            % Add the library directory to PATH
            addpath(fullfile(dynamixel_lib_path, 'c\include\dynamixel_sdk'));

            %Checks your OS and adds the correct built library
            %Pre-built for windows, you need to build yourself on Linux and
            %Mac. There is a Makefile in the linked folder.
            if strcmp(computer, 'PCWIN')
                obj.lib_name = 'dxl_x86_c';
                addpath(genpath(fullfile(dynamixel_lib_path, 'c\build\win32\')))
            elseif strcmp(computer, 'PCWIN64')
                addpath(genpath(fullfile(dynamixel_lib_path, 'c\build\win64\')))
                obj.lib_name = 'dxl_x64_c';
            elseif strcmp(computer, 'GLNX86')
                addpath(genpath(fullfile(dynamixel_lib_path, 'c\build\linux32\')))
                obj.lib_name = 'libdxl_x86_c';
            elseif strcmp(computer, 'GLNXA64')
                addpath(genpath(fullfile(dynamixel_lib_path, 'c\build\linux64\')))
                obj.lib_name = 'libdxl_x64_c';
            elseif strcmp(computer, 'MACI64')
                addpath(genpath(fullfile(dynamixel_lib_path, 'c\build\mac\')))
                obj.lib_name = 'libdxl_mac_c';
            end


            % Creating the proto file
            % % Change the current directory to where you want to save the protofile
            % oldFolder = cd(dynamixel_lib_path);
            % 
            % % Load the library and create the protofile in the specified directory
            % if ~libisloaded(obj.lib_name)
            %     [notfound, warnings] = loadlibrary(obj.lib_name, 'dynamixel_sdk.h', ...
            %                                        'addheader', 'port_handler.h', ...
            %                                        'addheader', 'packet_handler.h', ...
            %                                        'addheader', 'group_sync_read.h', ...
            %                                        'addheader', 'group_sync_write.h', ...
            %                                        'mfilename', 'dynamixel_proto');
            %     disp(warnings);
            % end
            % 
            % % Change back to the old directory
            % cd(oldFolder);
            loadlibrary(obj.lib_name, @dynamixel_proto);

            % Get Port Num
            obj.port_num = calllib(obj.lib_name, 'portHandler', port);

            % Initialize groupSyncWrite Struct
            obj.groupwrite_num = calllib(obj.lib_name, 'groupSyncWrite', obj.port_num, obj.PROTOCOL_VERSION, obj.ADDR_PRO_GOAL_VELOCITY, 4);
            obj.groupwrite_cur = calllib(obj.lib_name, 'groupSyncWrite', obj.port_num, obj.PROTOCOL_VERSION, obj.ADDR_PRO_GOAL_CURRENT, 2);

            % Initialize groupSyncRead Structs
            obj.groupread_num = calllib(obj.lib_name, 'groupSyncRead', obj.port_num, obj.PROTOCOL_VERSION, obj.ADDR_PRO_PRESENT_POSITION,4);
            obj.groupread_cur = calllib(obj.lib_name, 'groupSyncRead', obj.port_num, obj.PROTOCOL_VERSION, obj.ADDR_PRO_PRESENT_CURRENT,2);

            % Open port
            if (calllib(obj.lib_name, 'openPort', obj.port_num))
                fprintf('ServoChain: Succeeded to open the port!\n');
            else
                fprintf("ServoChain: Failed to open the port!\n");
                delete(obj)
                return
            end

            % Set baudrate
            if (calllib(obj.lib_name, 'setBaudRate', obj.port_num, obj.BAUDRATE))
                fprintf('ServoChain: Succeeded to change the baudrate!\n');
            else
                fprintf("ServoChain: Failed to change the baudrate to: %d\n", obj.BAUDRATE);
                delete(obj)
                return
            end

            % Initialize PacketHandler Structs
            calllib(obj.lib_name, 'packetHandler');

            %Scan available IDs
            fprintf('ServoChain: Scanning for Dynamixel from ID 0 to ID 10.. \n');
            calllib(obj.lib_name, 'broadcastPing', obj.port_num, obj.PROTOCOL_VERSION);
            fprintf('ServoChain: Detected Dynamixel : \n');
            for ID = 0 : 10
                if calllib(obj.lib_name, 'getBroadcastPingResult', obj.port_num, obj.PROTOCOL_VERSION, ID)
                    fprintf('ServoChain: Available ID: %d \n', ID);
                    % Store the available IDs
                    obj.availableIDs = [obj.availableIDs, ID];
                end
            end

            % After scanning for available servos set some settings
            for ID = obj.availableIDs
                obj.setServoTorque(ID, 0); % Set Torque Off
                obj.setOperationMode(ID, 1); % 1 = Velocity Control, 0 = Current Control
                obj.setDriveMode(ID, obj.TIME_BASED_PROFILE);  % Set Time-Based Profile
                obj.setProfileAcceleration(ID, 50);  % Set for Profile Acceleration to 300 ms acceleration time

                %If the servo ID is the yaw servo, also adjust the Velocity-I Gain and Velocity-P Gain
                if ID == 3
                    obj.setGains(ID, 5000, 600);
                else
                    obj.setGains(ID, 1920, 300);
                end

                obj.setServoTorque(ID, 1);
            end

            % Add present position of all available servos to groupSyncRead struct
            % Add goal velocity of all available servos to groupSyncWrite struct
            for ID = obj.availableIDs
                if ~ (calllib(obj.lib_name,'groupSyncReadAddParam', obj.groupread_num, ID) ...
                        && calllib(obj.lib_name,'groupSyncReadAddParam', obj.groupread_cur, ID) ...
                        && calllib(obj.lib_name,'groupSyncWriteAddParam', obj.groupwrite_num, ID, 0, 4) ...
                        && calllib(obj.lib_name,'groupSyncWriteAddParam', obj.groupwrite_cur, ID, 0, 2))

                    fprintf('ServoChain: groupSyncRead addparam failed');
                    delete(obj)
                    return
                end
            end
        end
    end

    methods

        %% Destructor
        function delete(obj)
            fprintf('ServoChain: Destructor called.\n')
            if  ~isempty(obj.port_num)
                if ~isempty(obj.groupread_num)
                    % Clear syncread parameter storage
                    calllib(obj.lib_name, 'groupSyncReadClearParam', obj.groupread_num);
                end
                if ~isempty(obj.groupread_cur)
                    % Clear syncread parameter storage
                    calllib(obj.lib_name, 'groupSyncReadClearParam', obj.groupread_cur);
                end
                if ~isempty(obj.groupwrite_num)
                    % Clear syncread parameter storage
                    calllib(obj.lib_name, 'groupSyncWriteClearParam', obj.groupwrite_num);
                end
                if ~isempty(obj.groupwrite_cur)
                    % Clear syncread parameter storage
                    calllib(obj.lib_name, 'groupSyncWriteClearParam', obj.groupwrite_cur);
                end
                % Try to close the port
                try
                    calllib(obj.lib_name, 'closePort', obj.port_num);
                    fprintf("ServoChain: Closed port.\n")
                catch
                    fprintf("ServoChain: Error Closing port.\n")
                end
            end
            % Unload library
            if ~isempty(obj.lib_name)
                try
                    unloadlibrary(obj.lib_name);
                    fprintf("ServoChain: Dynamixel Library unloaded.\n")
                catch
                    fprintf("ServoChain: Error unloading Dynamixel Library.\n")
                end
            end
        end

        %% Send/Receive
        function torqueEnabled = getServoTorque(obj, ID)
            % Check if the torque is enabled for a given servo ID.
            % Returns true if torque is enabled, false otherwise.
            
            % Read the torque enabled state
            torqueEnabled = calllib(obj.lib_name, 'read1ByteTxRx', obj.port_num, obj.PROTOCOL_VERSION, ID, obj.ADDR_PRO_TORQUE_ENABLE);
            obj.checkConnection();
        end
        
        function setServoTorque(obj,ID,state)
            % Enable / Disable the Torque of a servo.
            if(state)
                calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num, obj.PROTOCOL_VERSION, ID, obj.ADDR_PRO_TORQUE_ENABLE, 1);
            else
                calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num, obj.PROTOCOL_VERSION, ID, obj.ADDR_PRO_TORQUE_ENABLE, 0);
            end
            obj.checkConnection();
        end

        function servoAngles = getServoAngles(obj)

            % Syncread present position
            calllib(obj.lib_name,'groupSyncReadTxRxPacket', obj.groupread_num);

            num_servos = length(obj.availableIDs);
            servoAngles = zeros(num_servos,1);

            conversionFactor = 0.087891; % (see Dynamixel Wizard)
            maxrange = 2^(4*8)-1; % 4-byte range
            midpoint = maxrange/2;

            for i = 1:num_servos
                raw_angle = calllib(obj.lib_name, 'groupSyncReadGetData', obj.groupread_num, obj.availableIDs(i), obj.ADDR_PRO_PRESENT_POSITION, 4);
                % Check and Convert the position
                if raw_angle > midpoint
                    % Convert to a negative value
                    raw_angle = (raw_angle - maxrange) * conversionFactor;
                else
                    % Convert to a positive value
                    raw_angle = (raw_angle) * conversionFactor;
                end
                servoAngles(i) = deg2rad(raw_angle);
            end
            obj.checkConnection();
        end

        function setServoVelocities(obj, servoVelocities)

            % The servo velocities will be set with the same order as the
            % available IDs
            num_servos = length(obj.availableIDs);

            if ~(length(servoVelocities) == num_servos)
                fprintf("Error setting servo velocities: \nSize of servoVelocities does not match number of available IDs!\n");
                return;
            end
            for i = 1:num_servos
                if ~(obj.getServoOperationMode(i) == 1)
                    fprintf("Warning: \nServo %.0f not set to correct operation mode\n", i);
                end
            end

            % Convert desired rev/min to dynamixel decimal
            VELOCITIES_VAL = (servoVelocities)/0.229; % Convert rev/min to decimal

            %Round VELOCITY_VAL since dynamixel accepts only integers here
            VELOCITIES_VAL = round(VELOCITIES_VAL);

            % VELOCITY_VAL is a value in 4 byte (256^4) continuous range
            % A value of (256^4) / 2 is zero, values bigger are positive,
            % values smaller are negative.
            maxrange = 2^(4*8)-1; % 4-byte range

            % Change goal velocities of all available servos in groupSyncWrite struct
            for i = 1:num_servos
                % Add parameter storage for present position value
                VEL = VELOCITIES_VAL(i);
                if VEL < 0
                    VEL = maxrange+VEL;
                end
                calllib(obj.lib_name,'groupSyncWriteChangeParam', obj.groupwrite_num, obj.availableIDs(i), VEL, 4,0);
            end

            % Syncwrite goal position
            calllib(obj.lib_name,'groupSyncWriteTxPacket',obj.groupwrite_num);
            obj.checkConnection();
        end

        function blink(obj, ID)

            obj.toggleServoLED(ID);
            pause(0.5);             % 0.5 seconds
            obj.toggleServoLED(ID);
        end
        
        function setServoOperationMode(obj, ID, operationMode)

            % mode = 1: velocity control
            % mode = 0: current control

            if obj.getServoTorque(ID)
                fprintf('Error setting operation mode: disable torque first!\n');
                return;
            end

            obj.setOperationMode(ID, operationMode);

        end

        function servoMode = getServoOperationMode(obj, ID)

            servoMode = calllib(obj.lib_name, 'read1ByteTxRx', obj.port_num, obj.PROTOCOL_VERSION, ID, obj.ADDR_PRO_OPERATION_MODE);
            obj.checkConnection();

        end

        function servoCurrents = getServoCurrents(obj) %doesnt work yet!
            
            % Syncread present currents
            calllib(obj.lib_name,'groupSyncReadTxRxPacket', obj.groupread_cur);

            num_servos = length(obj.availableIDs);
            servoCurrents = zeros(num_servos,1);

            conversionFactor = 2.69; % (see Dynamixel Wizard)
            maxrange = 2^(2*8)-1; % 2-byte range
            midpoint = maxrange/2;

            for i = 1:num_servos
                raw_current = calllib(obj.lib_name, 'groupSyncReadGetData', obj.groupread_cur, obj.availableIDs(i), obj.ADDR_PRO_PRESENT_CURRENT, 2);
                if raw_current > midpoint
                    raw_current = raw_current - maxrange;
                end
                servoCurrents(i) = raw_current * conversionFactor;
            end
            obj.checkConnection();
        end

        function setServoCurrents(obj, servoCurrents)

            % The servo currents will be set with the same order as the
            % available IDs
            num_servos = length(obj.availableIDs);

            if ~(length(servoCurrents) == num_servos)
                fprintf("Error setting servo currents: \nSize of servoCurrents does not match number of available IDs!\n");
                return;
            end
            for i = 1:num_servos
                if ~(obj.getServoOperationMode(i) == 0)
                    fprintf("Warning: \nServo %.0f not set to correct operation mode\n", i);
                end
            end

            % Convert desired mA to dynamixel decimal
            CURRENTS_VAL = (servoCurrents)/2.69; % Convert mA to decimal

            %Round CURRENTS_VAL since dynamixel accepts only integers here
            CURRENTS_VAL = round(CURRENTS_VAL);

            % VELOCITY_VAL is a value in 2 byte (256^2) continuous range
            % A value of (256^2) / 2 is zero, values bigger are positive,
            % values smaller are negative.
            maxrange = 2^(2*8)-1; % 2-byte range

            % Change goal currents of all available servos in groupSyncWrite struct
            for i = 1:num_servos
                % Add parameter storage for present position value
                CUR = CURRENTS_VAL(i);
                if CUR < 0
                    CUR = maxrange+CUR;
                end
                calllib(obj.lib_name,'groupSyncWriteChangeParam', obj.groupwrite_cur, obj.availableIDs(i), CUR, 2,0);
            end

            % Syncwrite goal position
            calllib(obj.lib_name,'groupSyncWriteTxPacket',obj.groupwrite_cur);
            obj.checkConnection();
        end


    end

    methods (Access = private)
        %% Check the communication after a Send/Receive call, Calls destructor if failed
        function checkConnection(obj)
            dxl_comm_result = calllib(obj.lib_name, 'getLastTxRxResult', obj.port_num, obj.PROTOCOL_VERSION);
            if dxl_comm_result ~= obj.COMM_SUCCESS % --> failed connection
                fprintf('ServoChain: Connection Failed!\n');
                fprintf('%s\n', calllib(obj.lib_name,'getTxRxResult', obj.PROTOCOL_VERSION, dxl_comm_result));
                delete(obj)
                return
            end
        end


      function setGains(obj, ID, velocityIGain, velocityPGain)
            % Set the Velocity-I Gain and Velocity-P Gain of a specific servo

            % Write Velocity-I Gain
            calllib(obj.lib_name, 'write2ByteTxRx', obj.port_num, obj.PROTOCOL_VERSION, ID, obj.ADDR_PRO_VELOCITY_I_GAIN, velocityIGain);
            obj.checkConnection();

            % Write Velocity-P Gain
            calllib(obj.lib_name, 'write2ByteTxRx', obj.port_num, obj.PROTOCOL_VERSION, ID, obj.ADDR_PRO_VELOCITY_P_GAIN, velocityPGain);
            obj.checkConnection();
      end

      function setOperationMode(obj, ID, operationMode)
            % Set the Operation Mode for a specific servo
            calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num, obj.PROTOCOL_VERSION, ID, obj.ADDR_PRO_OPERATION_MODE, operationMode);
            obj.checkConnection();
      end



        function setDriveMode(obj, ID, modeValue)
            % Set Drive Mode to Time-Based Profile
            calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num, obj.PROTOCOL_VERSION, ID, obj.ADDR_PRO_DRIVE_MODE, modeValue);
            obj.checkConnection();
        end
    
        function setProfileAcceleration(obj, ID, acceleration)
            % Set Profile Acceleration
            calllib(obj.lib_name, 'write4ByteTxRx', obj.port_num, obj.PROTOCOL_VERSION, ID, obj.ADDR_PRO_PROFILE_ACCELERATION, acceleration);
            obj.checkConnection();
        end

        function setServoLED(obj,ID,state)
            % Enable / Disable the Torque of a servo.
            if(state)
                calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num, obj.PROTOCOL_VERSION, ID, obj.ADDR_PRO_LED, 1);
            else
                calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num, obj.PROTOCOL_VERSION, ID, obj.ADDR_PRO_LED, 0);
            end
            obj.checkConnection();
        end
   
        function toggleServoLED(obj, ID)

            state = calllib(obj.lib_name, 'read1ByteTxRx', obj.port_num, obj.PROTOCOL_VERSION, ID, obj.ADDR_PRO_LED);
            calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num, obj.PROTOCOL_VERSION, ID, obj.ADDR_PRO_LED, ~state);

            obj.checkConnection();
        end
    
    end
end

