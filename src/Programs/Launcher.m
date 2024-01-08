classdef Launcher < handle
    properties
        virtualRobot;
        realRobot;
        currentProgramInstance;
        programNames;
        status;
    end
   
    methods
        

        function obj = Launcher()

            % Add all relevant folders to MATLAB PATH
            obj.initPath();
            
            % No connection upon launcher intialisation
            obj.realRobot = [];

            % Create the Virtual Robot Object
            obj.virtualRobot = VirtualRobot;

            % Load all available Programs
            obj.programNames = obj.getPrograms;
            obj.currentProgramInstance = [];

            obj.status = 'disconnected';
            % 'disconnected' --> no realRobot object, no connection,
            % 'ready' --> connection to real robot, no program running
            % 'singularity' --> plotting method warns singularity config
            % 'executing' --> non-plotting program executing

        end

        function delete(obj)
            delete(obj.currentProgramInstance)

        end

        function connect(obj,varargin)
            % Disconnect first (if a connection exists)
            obj.disconnect;

            % Check if the constructor was called with a specified port
            if nargin < 2
                % If not, use 'COM3' as a default port
                PORT = 'COM3';
            else
                % Use the provided port
                PORT = varargin{1};
            end

            % Get Path to Dynamixel Lib
            currentFile = mfilename('fullpath');
            [currentDir, ~, ~] = fileparts(currentFile);
            [parentDir, ~, ~] = fileparts(currentDir);
            dynamixel_lib_path = fullfile(parentDir, 'DynamixelLib\c\');

            % Connect
            obj.realRobot = RealRobot(dynamixel_lib_path,PORT);

            % Set zero pos
            obj.realRobot.setZeroPositionToCurrentPosition;

            % Start the plotting program
            obj.instantiateProgram('Plotting')
            obj.currentProgramInstance.start();

            % Set the Torque to enabled
            obj.realRobot.torqueEnable;
            fprintf("Launcher: Torque Enabled. \n");
        end

        function disconnect(obj)
            % Stop any program running
            % Stop the current program if there is one
            if ~isempty(obj.currentProgramInstance)
                delete(obj.currentProgramInstance)
            end
            % Disconnect
            delete(obj.realRobot);
            obj.status = 'disconnected';
        end
        
        function instantiateProgram(obj, programName)
            % Check if the programName is in the available programs
            if ismember(programName, obj.programNames)

                % Stop the current program if there is one
                if ~isempty(obj.currentProgramInstance)
                    delete(obj.currentProgramInstance)
                end

                % Instantiate the program using its name
                % Pass a reference to the launcher instance and variable
                % arguments
                try
                    obj.currentProgramInstance = feval(programName, obj);
                    fprintf('Launcher: Insantiated Program: %s \n', programName);
                catch ME
                    fprintf('Launcher: Failed to instantiate %s. Error: %s. \n', programName, ME.message );
                end        
            else
                fprintf('Launcher: Program %s not found in available programs. \n', programName);
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

        function initPath(~)
            % Initialize the MATLAB path

            clc;
            close all;

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

        end
    end
end
