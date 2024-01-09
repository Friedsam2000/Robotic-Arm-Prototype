classdef Launcher < handle
    properties
        % Instances
        virtualRobot;
        realRobot;
        currentProgramInstance;

        % Singularity warning: true or false
        singularityWarning;

        % List of all available concrete programs
        programNames;

        % Launcher Status:
        % 'disconnected' --> no realRobot object, no connection,
        % 'ready' --> connection to real robot, no program running
        % 'busy' --> non-plotting program executing
        status;
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

            clc
            close all

            % Add all relevant folders to MATLAB PATH
            dynamixel_lib_path = obj.initPath();

            % Create the Virtual Robot Object
            obj.virtualRobot = VirtualRobot;

            % Load all available Programs
            obj.programNames = obj.getPrograms;
            obj.currentProgramInstance = [];

            obj.status = 'disconnected';

            % Check if the constructor was called with a specified port
            if nargin < 1
                % If not, use 'COM3' as a default port
                PORT = 'COM3';
            else
                % Use the provided port
                PORT = varargin{1};
            end

            % Connect
            obj.realRobot = RealRobot(dynamixel_lib_path,PORT);

            % Check Connection
            if ~obj.realRobot.servoChain.checkConnection
                error("Connection Failed")
            end

            % Set zero pos
            obj.realRobot.setZeroPositionToCurrentPosition;
            obj.singularityWarning = true;

            % Set status
            obj.status = 'ready';

            % Start the plotting program
            obj.launchProgram('Plotting')

            % Set the Torque to enabled
            obj.realRobot.torqueEnable;
            fprintf("Launcher: Torque Enabled. \n");
        end
    end

    methods
        function delete(obj)

            % Stop any running program
            delete(obj.currentProgramInstance);

            % Return to zero
            obj.realRobot.torqueEnable;
            fprintf('Launcher: Returning To Zero. \n')
            obj.launchProgram('Set_Joints', [0;0;0;0]);

            % Waint until program finished
            while ~strcmp(obj.status, 'ready')
                pause(0.1); % Short pause to yield execution
                drawnow;    % Process any pending callbacks or events
            end

            fprintf('Launcher: Stopping \n');
            obj.realRobot.setJointVelocities([0;0;0;0]);

            % Stop any running program
            delete(obj.currentProgramInstance);

            % Disconnect
            delete(obj.realRobot); 

            close all
        end

        function launchProgram(obj, programName, varargin)


            % Check if in ready state
            if ~strcmp(obj.status, 'ready')
                fprintf("Launcher: Can't Launch Program. Status: %s \n", obj.status);
                return;
            end

            % Check if program is in the list of available programs
            if ~ismember(programName, obj.programNames)
                fprintf('Launcher: Program %s not found in available programs. \n', programName);
                return
            end

            % Stop any running program
            delete(obj.currentProgramInstance);


            % Try to launch
            try
                obj.status = 'busy';
                onDeleteCallback = @(progObj) obj.onProgramDelete(progObj);
                obj.currentProgramInstance = feval(programName, obj, onDeleteCallback);
                fprintf('Launcher: Program %s starting...\n', class(obj.currentProgramInstance));
                obj.currentProgramInstance.start(varargin{:});
                if isa(obj.currentProgramInstance, 'Plotting')
                    obj.status = 'ready';
                end
            catch ME
                fprintf('Launcher: Failed to launch Program: %s. Error: %s. \n', programName, ME.message);
                obj.status = 'ready';
                return;
            end
        end

        function onProgramDelete(obj, deletedProgram)
            % Callback function to handle program deletion
            fprintf('Launcher: Program %s ended.\n', class(deletedProgram));
            if ~isa(deletedProgram, 'Plotting')
                % Set the status to 'ready' if the deleted program was not
                % the plotting program
                obj.status = 'ready';
                % Relaunch plotting
                obj.launchProgram('Plotting')
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

            dynamixel_lib_path = fullfile(parentDir, 'DynamixelLib\c\');
        end
    end
end
