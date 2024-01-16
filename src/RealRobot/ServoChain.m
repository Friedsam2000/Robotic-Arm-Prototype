classdef ServoChain < handle

    properties (Constant)
        PROTOCOL_VERSION = 2;
    end

    properties

        % IDs of available Dynamixel Servos 
        availableIDs = [];

        lib_name;
        port_num;

    end

    methods
        %% Constructor = Connection Attempt, Calls destructor if failed
        function obj = ServoChain(dynamixel_lib_path, port)

            % Add the include directory
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

            % Load Libraries
            if ~libisloaded(obj.lib_name)
                [notfound, warnings] = loadlibrary(obj.lib_name, 'dynamixel_proto');
                disp(warnings);
                if isempty(notfound) && isempty(warnings)
                    fprintf("Succeeded to load the dynamixel library \n");
                else
                    frpintf("Failed to load the dynamixel library \n");
                    delete(obj)
                    return
                end
            end

            %Local Definitions
            MAX_ID = 10;
            BAUDRATE = 1000000;

            % Open port
            obj.port_num = calllib(obj.lib_name, 'portHandler', port);
            if (calllib(obj.lib_name, 'openPort', obj.port_num))
                fprintf('ServoChain: Succeeded to open the port!\n');
            else
                fprintf("ServoChain: Failed to open the port!\n");
                delete(obj)
                return
            end

            % Set port baudrate
            if (calllib(obj.lib_name, 'setBaudRate', obj.port_num, BAUDRATE))
                fprintf('ServoChain: Succeeded to change the baudrate!\n');
            else
                fprintf("ServoChain: Failed to change the baudrate!\n");
                delete(obj)
                return
            end

            % Initialize PacketHandler Structs
            calllib(obj.lib_name, 'packetHandler');

            %Scan available IDs
            % Try to broadcast ping the Dynamixel
            calllib(obj.lib_name, 'broadcastPing', obj.port_num, obj.PROTOCOL_VERSION);

            fprintf('ServoChain: Detected Dynamixel : \n');
            for ID = 0 : MAX_ID
                if calllib(obj.lib_name, 'getBroadcastPingResult', obj.port_num, obj.PROTOCOL_VERSION, ID)
                    fprintf('ServoChain: Available ID: %d \n', ID);
                    % Store the available IDs
                    obj.availableIDs = [obj.availableIDs, ID];
                end
            end
            if length(obj.availableIDs) < 4
                fprintf("ServoChain: Not all 4 dynamixel servos detected!\n")
                delete(obj)
                return
            else
                fprintf("ServoChain: All Servos Detected. Ready.\n")
            end

        end

        %% Destructor = Closing Port
        function delete(obj)
            fprintf("ServoChain: Closing port.\n")
            try
                calllib(obj.lib_name, 'closePort', obj.port_num);
            end
        end

        %% Send/Receive 

        function servoAngle = getServoAngle(obj,ID)
            %Receive the current Position of a servo in RAD. Can be multi
            %rotation and supports negative angles.

            %Local Definitions
            ADDR_PRO_PRESENT_POSITION    = 132;

            % Get the present position
            % This should give a value in 4 byte (256^4) continuous range
            dxl1_present_position = calllib(obj.lib_name, 'read4ByteTxRx', obj.port_num , obj.PROTOCOL_VERSION, ID, ADDR_PRO_PRESENT_POSITION);

            % Define the conversion factor and midpoint
            conversionFactor = 0.087891; % (see Dynamixel Wizard)

            maxrange = 2^(4*8)-1; % 4-byte range
            midpoint = maxrange/2;

            % Check and Convert the position
            if dxl1_present_position > midpoint
                % Convert to a negative value
                servoAngle = (dxl1_present_position - maxrange) * conversionFactor;
            else
                % Convert to a positive value
                servoAngle = (dxl1_present_position) * conversionFactor;
            end
            % Convert to RAD
            servoAngle = deg2rad(servoAngle);

        end

        function setServoVelocity(obj, ID, servoVelocity)

            % Set a Servos velocity in rev/min

            % Local Definitions
            ADDR_PRO_GOAL_VELOCITY      = 104;

            % Convert desired rev/min to dynamixel decimal
            VELOCITY_VAL = (servoVelocity)/0.229; % Convert rev/min to decimal

            %Round VELOCITY_VAL since dynamixel accepts only integers here
            VELOCITY_VAL = round(VELOCITY_VAL);

            % VELOCITY_VAL is a value in 4 byte (256^4) continuous range
            % A value of (256^4) / 2 is zero, values bigger are positive,
            % values smaller are negative.
            maxrange = 2^(4*8)-1; % 4-byte range
            if VELOCITY_VAL < 0
                VELOCITY_VAL = maxrange+VELOCITY_VAL;
            end

            calllib(obj.lib_name, 'write4ByteTxRx', obj.port_num , obj.PROTOCOL_VERSION, ID, ADDR_PRO_GOAL_VELOCITY, VELOCITY_VAL);
        end
    
        function setServoTorque(obj,ID,state)
            % Enable / Disable the Torque of a servo.

            %Local Definitions
            ADDR_PRO_TORQUE_ENABLE       = 64;         % Control table address is different in Dynamixel model
            TORQUE_ENABLE               = 1;            % Value for enabling the torque
            TORQUE_DISABLE              = 0;            % Value for disabling the torque

            % Enable / Disable torque
            if(state)
                calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num, obj.PROTOCOL_VERSION, ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
            else
                calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num, obj.PROTOCOL_VERSION, ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
            end
        end

        function isConnected = checkConnection(obj)
            VALUE = calllib(obj.lib_name, 'pingGetModelNum', obj.port_num, obj.PROTOCOL_VERSION, 4);
            if VALUE ~= 1010
                isConnected = false;
            else
                isConnected = true;
            end
        end

    end
end

