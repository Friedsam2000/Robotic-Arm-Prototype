classdef Launcher < handle
    properties
        % Instances
        virtualRobot;
        realRobot; % if empty -> not connected
        currentProgramInstance; % if empty -> not running

        % Singularity warning: true or false
        singularityWarning;

        % List of all available concrete programs
        programNames;

        % ConfigUpdateTimer
        configUpdateTimer;

        % An optional reference to a matlab app which controls the launcher
        matlabAppObj;

    end


    methods (Static)
        % Static method to get the instance of the class
        function single_instance = getInstance(varargin)
            persistent instance;
            if isempty(instance) || ~isvalid(instance)
                instance = Launcher(varargin{:});
            end

            single_instance = instance;
        end
    end


    methods (Access = private)

        function obj = Launcher(varargin)

            % Optionally Link a Matlab app
            if nargin < 1
                obj.matlabAppObj = [];
            else
                % Use the provided port
                obj.matlabAppObj = varargin{1};
            end

            % Add all relevant folders to MATLAB PATH
            Launcher.initPath;

            % Create the Virtual Robot Object
            obj.virtualRobot = VirtualRobot;

            % Create the configUpdateTimer
            obj.configUpdateTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.1, ...
                'BusyMode', 'drop', 'TimerFcn', @(~,~) obj.updateConfigAndPlot);

            % Load all available Programs
            obj.programNames = obj.getPrograms;
            obj.currentProgramInstance = [];

        end
    end

    methods

        function connect(obj, varargin)
            % Check if the constructor was called with a specified port
            if nargin < 2
                % If not, use 'COM3' as a default port
                PORT = 'COM3';
            else
                % Use the provided port
                PORT = varargin{1};
            end

            % Connect
            dynamixel_lib_path = Launcher.initPath;
            obj.realRobot = RealRobot(dynamixel_lib_path,PORT);

            % Check Connection
            if ~obj.realRobot.servoChain.checkConnection
                warning("Launcher: Connection Failed on USB Port %s \n", PORT);
            else
                fprintf("Launcher: Successfully connected on USB Port %s \n", PORT);

                % Set Zero Positoin
                obj.realRobot.setZeroPositionToCurrentPosition;

                obj.singularityWarning = true;
                fprintf("Launcher: Zero Position Set. \n");

                % Start the Plotting timer
                fprintf("Launcher: Starting Plotting Timer. \n");
                start(obj.configUpdateTimer);
            end

        end

        function disconnect(obj)

            fprintf("Launcher: Disconnecting... \n")

            % Stop any running program
            delete(obj.currentProgramInstance);
            obj.currentProgramInstance = [];
            % If the launcher is connected
            if ~isempty(obj.realRobot)

                % Stop the UpdateConfigTimer
                fprintf("Launcher: Stopping Plotting Timer. \n");
                stop(obj.configUpdateTimer);

                % Break connection by deleting realRobot object
                delete(obj.realRobot)
                obj.realRobot = [];
            end
        end

        function delete(obj)
            obj.disconnect;
            fprintf("Launcher: Deleting Plotting Timer.\n")
            delete(obj.configUpdateTimer);
            obj.configUpdateTimer = [];
            fprintf("Launcher: Deleting.\n")
        end

        function launchProgram(obj, programName, varargin)


            % Check if program is in the list of available programs
            if ~ismember(programName, obj.programNames)
                fprintf('Launcher: Program %s not found in available programs. \n', programName);
                return
            end

            % Stop and Delete any running program
            delete(obj.currentProgramInstance);

            % Stop the configUpdateTimer
            fprintf("Launcher: Stopping Plotting Timer. \n");
            stop(obj.configUpdateTimer)

            % Create and Store an Instance of the Program and pass the
            % launcher object by reference
            obj.currentProgramInstance = feval(programName, obj);

            fprintf('Launcher: Program %s starting...\n', class(obj.currentProgramInstance));

            % Try to launch
            try
                % Start the Program
                obj.currentProgramInstance.start(varargin{:});
            end

        end

        function programDeleteCallback(obj)
            % Callback function that gets called by the program after
            % stopping or error and before it delets itself
            fprintf('Launcher: Program ended.\n');

            % Clear the reference to it
            obj.currentProgramInstance = [];

            % Stop
            obj.realRobot.setJointVelocities([0;0;0;0])

            % Restart the plotting timer
            fprintf("Launcher: Starting Plotting Timer. \n");
            start(obj.configUpdateTimer);

        end

        function updateConfigAndPlot(obj)
            % Check if figure is still valid
            if isempty(obj.virtualRobot.fig) || ~isvalid(obj.virtualRobot.fig)
                % If not reopen the plot
                obj.virtualRobot.initRobotPlot;
            end

            % Common method to update the virtual robots configuration and
            % update the plot
            obj.virtualRobot.setQ(obj.realRobot.getQ);
            obj.virtualRobot.updateRobotPlot;

            % Update Singularity Status
            obj.singularityWarning = obj.virtualRobot.checkSingularity;

            if ~isempty(obj.matlabAppObj)
                obj.matlabAppObj.updateConfigCallback;
            end
        end

        function [is_valid, error_msg] = checkProgramArgs(~,programName, arguments)

            error_msg = [];
            switch programName
                case 'Set_Joints'
                    % Expecting 4 doubles (including negatives) separated
                    % by commas or semicolons
                    expression = '^(-?\d+(\.\d+)?[,;] *){3}-?\d+(\.\d+)?$';
                    if regexp(arguments, expression)
                        is_valid = true;
                    else
                        is_valid = false;
                        error_msg = 'Please enter 4 joint angles (including negatives) separated by commas or semicolons.';
                    end

                case 'Set_Position'
                    % Expecting 3 doubles (including negatives) separated
                    % by commas or semicolons
                    expression = '^(-?\d+(\.\d+)?[,;] *){2}-?\d+(\.\d+)?$';
                    if regexp(arguments, expression)
                        is_valid = true;
                    else
                        is_valid = false;
                        error_msg = 'Please enter 3 position coordinates (x, y, z) including negatives, separated by commas or semicolons.';
                    end
                case 'Trajectory_2D'
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
    end


    methods (Static, Hidden)


        function programs = getPrograms()
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

        function dynamixel_lib_path = initPath()

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

            dynamixel_lib_path = fullfile(parentDir, 'DynamixelLib\c');
        end
    end
end
