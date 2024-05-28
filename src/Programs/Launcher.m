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

            % Clear all timers
            try
                timers = timerfindall;
                delete(timers);
            end


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
                'BusyMode', 'drop', 'TimerFcn', @(~,~) obj.syncJointsAndPlot, 'ErrorFcn', @(~,~) obj.disconnect);

        end

        %% Destructor
        function delete(obj)

            if ~isempty(obj.realRobot) && isvalid(obj.realRobot)
                obj.disconnect;
            end

            fprintf("Launcher: Deleting jointSyncTimer.\n")
            delete(obj.jointSyncTimer);
            obj.jointSyncTimer = [];

            fprintf("Launcher: Deleting.\n")
        end

        %% Connection Management
        function success = connect(obj, varargin)
            success = true;
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
                success = false;
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

            % Update the App if specified
            if ~isempty(obj.matlabApp)
                obj.matlabApp.updateGUI;
            end
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

            % If connection was lost during the program, dont restart the
            % jointSyncTimer and optionally update the App
            if ~obj.checkConnection()
                if ~isempty(obj.matlabApp)
                    obj.matlabApp.updateGUI;
                end
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
                obj.matlabApp.updateGUI;
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
        function [is_valid, error_msg] = checkProgramArgs(~, programName, arguments)
            error_msg = [];
            is_valid = false;  % Default to false
        
            % Fetch arguments information from the program
            argsInfo = feval([programName '.getArgumentsInfo']);
            
            % Use the validation pattern from argsInfo
            if regexp(arguments, argsInfo.validationPattern)
                % Continue with your existing validation logic here,
                % potentially customizing it based on additional info from argsInfo if necessary
                is_valid = true;
            else
                error_msg = ['Invalid input. Expected: ' argsInfo.placeholder];
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
            if ~obj.realRobot.checkConnection()
                state = false;
                obj.realRobot = [];
                return;
            end
        end
    end
end
