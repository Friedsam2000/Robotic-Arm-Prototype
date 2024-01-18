classdef Launcher < handle
    properties

        jointSyncTimer = [];
        virtualRobot = [];
        realRobot = [];     
        activeProgram = [];

        % An optional reference to a matlab app which controls the launcher
        matlabApp = [];
    end


    methods

        %% Constructor
        function obj = Launcher(varargin)

            % Optionally Link a Matlab app
            if nargin == 1
                obj.matlabApp = varargin{1};
            end

            % Add all relevant folders to MATLAB PATH
            obj.initPath;

            % Create the Virtual Robot Object
            obj.virtualRobot = VirtualRobot;

            % Create the jointSyncTimer
            obj.jointSyncTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.1, ...
                'BusyMode', 'drop', 'TimerFcn', @(~,~) obj.syncJointsAndPlot, 'ErrorFcn', @(~,~) obj.delete);

        end

        %% Destructor
        function delete(obj)

            obj.disconnect;

            fprintf("Launcher: Deleting jointSyncTimer.\n")
            delete(obj.jointSyncTimer);
            obj.jointSyncTimer = [];

            fprintf("Launcher: Deleting.\n")
        end

        %% Connection Management
        function connect(obj, varargin)
            % Check if the constructor was called with a specified port
            if nargin < 2
                % If not, use 'COM3' as a default port
                port = 'COM3';
            else
                % Use the provided port
                port = varargin{1};
            end

            % Connect
            dynamixel_lib_path = obj.initPath;
            obj.realRobot = RealRobot(dynamixel_lib_path,port);

            if ~obj.checkConnection()
                return;
            end

            fprintf("Launcher: Successfully connected on USB Port %s \n", port);

            % Set Zero Position
            obj.realRobot.zeroAtCurrent;
            fprintf("Launcher: Zero Position Set. \n");

            % Enable torque
            obj.realRobot.setRobotTorque(1);
            fprintf("Launcher: Starting Plotting Timer. \n");

            % Start the Plotting timer
            start(obj.jointSyncTimer);
        end

        function disconnect(obj)
            fprintf("Launcher: Disconnecting... \n")

            % Stop any running program
            delete(obj.activeProgram);
            obj.activeProgram = [];

            % Stop the jointSyncTimer
            fprintf("Launcher: Stopping jointSyncTimer. \n");
            stop(obj.jointSyncTimer);

            % Break connection by deleting realRobot object
            delete(obj.realRobot)
            obj.realRobot = [];
        end

        %% Program
        function launchProgram(obj, programName, varargin)

            if ~obj.checkConnection()
                return;
            end

            % Stop and Delete any running program
            delete(obj.activeProgram);

            % Stop the jointSyncTimer
            fprintf("Launcher: Stopping Plotting Timer. \n");
            stop(obj.jointSyncTimer);

            % Instantiate the Program
            obj.activeProgram = feval(programName, obj);

            % Start the Program
            fprintf('Launcher: Program %s starting...\n', class(obj.activeProgram));
            obj.activeProgram.start(varargin{:});

        end

        function programTerminationCallback(obj)
            % Callback function that gets called by the program after
            % stopping or error and before it delets itself
            fprintf('Launcher: Program ended.\n');

            % Clear the reference to the activeProgram
            obj.activeProgram = [];

            if ~obj.checkConnection()
                return;
            end

            % Stop any Motion
            obj.realRobot.setJointVelocities([0;0;0;0])

            % Restart the plotting timer
            fprintf("Launcher: Starting Plotting Timer. \n");
            start(obj.jointSyncTimer);


        end

        function syncJointsAndPlot(obj)

            if ~obj.checkConnection()
                return;
            end

            % Sync Joints of virtualRobot and realRobot
            obj.virtualRobot.setJointAngles(obj.realRobot.getJointAngles);

            % Update the App if specified
            if ~isempty(obj.matlabApp)
                obj.matlabApp.updateCallback;
            end

            % Plotting
            % Check if figure is still valid
            if isempty(obj.virtualRobot.fig) || ~isvalid(obj.virtualRobot.fig)
                % If not reopen the plot
                obj.virtualRobot.initRobotPlot;
            else
                obj.virtualRobot.updateRobotPlot;
            end

            drawnow limitrate;
        end


        %% Utility
        function [is_valid, error_msg] = checkProgramArgs(obj, programName, arguments)
            error_msg = [];
            is_valid = false;  % Default to false

            switch programName
                case 'SetJoints'
                    % Expecting 4 doubles (including negatives) separated by commas or semicolons
                    expression = '^(-?\d+(\.\d+)?[,;] *){3}-?\d+(\.\d+)?$';
                    if regexp(arguments, expression)
                        % Split the arguments into an array of numbers
                        joint_angles = str2double(split(regexprep(arguments, '[,;]', ' ')));

                        % Check if the joint angles are within the limits
                        joint_limits = rad2deg(obj.virtualRobot.JOINT_ANGLE_LIMITS);
                        if all(joint_angles >= joint_limits(:,1)) && all(joint_angles <= joint_limits(:,2))
                            is_valid = true;
                        else
                            % Convert joint limits to degrees and make them positive
                            positive_joint_limits_deg = joint_limits(:,2);
                            % Format the limits as integer values
                            % Convert to a flat char array with spaces
                            formatted_limits = sprintf('%d,', positive_joint_limits_deg(:));

                            error_msg = ['The joint limits are: ' formatted_limits(1:end-1), ' °'];
                        end
                    else
                        error_msg = 'Please enter 4 joint angles (including negatives) separated by commas or semicolons.';
                    end

                case 'SetPosition'
                    % Expecting 3 doubles (including negatives) separated
                    % by commas or semicolons
                    expression = '^(-?\d+(\.\d+)?[,;] *){2}-?\d+(\.\d+)?$';
                    if regexp(arguments, expression)
                        is_valid = true;
                    else
                        is_valid = false;
                        error_msg = 'Please enter 3 position coordinates (x, y, z) including negatives, separated by commas or semicolons.';
                    end
                case 'Trajectory2D'
                    % Expecting a string that can be converted to a single double
                    % First, try to convert the argument to a numeric value
                    num = str2double(arguments);
                    if ~isnan(num) && isscalar(num)
                        is_valid = true;
                    else
                        is_valid = false;
                        error_msg = 'Please enter a valid numeric value.';
                    end
            end
        end

        function programs = getPrograms(~)
            % Get Current Dir of Launcher.m file
            currentFile = mfilename('fullpath');
            [currentDir, ~, ~] = fileparts(currentFile);

            % Construct the path to the 'ConcretePrograms' folder
            ConcreteProgramsPath = fullfile(currentDir, 'ConcretePrograms');

            % Load program names
            programFiles = dir(fullfile(ConcreteProgramsPath, '*.m'));
            programs = {}; % Initialize empty cell array for program names

            for i = 1:length(programFiles)
                programName = erase(programFiles(i).name, '.m');
                programs{end + 1} = char(programName); % Add the program name to the list
            end
        end

        function dynamixel_lib_path = initPath(~)

            % Get Current Dir of Launcher.m file (Programs)
            currentFile = mfilename('fullpath');
            [currentDir, ~, ~] = fileparts(currentFile);

            % Get Parent Dir (src)
            [parentDir, ~, ~] = fileparts(currentDir);

            addpath(fullfile(parentDir, 'VirtualRobot'));
            addpath(fullfile(parentDir, 'RealRobot'));
            addpath(fullfile(parentDir, 'Planner'));
            addpath(fullfile(parentDir, 'Controller'));
            addpath(fullfile(parentDir, 'Programs'));
            addpath(fullfile(parentDir, 'Programs\ConcretePrograms'));
            addpath(fullfile(parentDir, 'DynamixelLib'));

            dynamixel_lib_path = fullfile(parentDir, 'DynamixelLib');
        end
    end

    
    methods (Access = private)

        function state = checkConnection(obj)
            state = true;
            if isempty(obj.realRobot) || ~isvalid(obj.realRobot)
                state = false;
                obj.realRobot = [];
                return;
            end
        end
    end
end
