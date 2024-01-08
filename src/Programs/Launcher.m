classdef Launcher < handle
    properties (SetAccess=private)
        virtualRobot = [];
        realRobot = [];
        currentProgramInstance = [];
        programNames = [];
    end

    methods (Static)

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
                if ~exist(programName, 'class')
                    continue; % Skip if the class does not exist
                end

                % Attempt to access the constant 'name' property of the class
                try
                    programName = eval([programName '.name']);
                    if ischar(programName) || isstring(programName)
                        programs{end + 1} = char(programName); % Add the program name to the list
                    end
                catch
                    % Handle or ignore errors in class instantiation
                end
            end
        end

    end

    methods
        function obj = Launcher()
            % Constructor (can only be accessed via getInstance method
            % This is to ensure only one launcher is active at a time
            % (unique)

            % Add all relevant folders to MATLAB PATH
            obj.initPath();

            % Create the Virtual Robot Object
            obj.virtualRobot = VirtualRobot;

            % Load all available Programs
            obj.programNames = obj.getPrograms;

        end

    end

    methods

        function delete(obj)
            % Destructor
            obj.disconnect;
            delete(obj.virtualRobot);
        end

        function connect(obj,varargin)

            % Reconnect if Connect is called as realRobot exists.
            if ~isempty(obj.realRobot)
                delete(obj.realRobot)
            end

            % Check if the constructor was called with a specified port
            if nargin < 2
                % If not, use 'COM3' as a default port
                PORT = 'COM3';
            else
                % Use the provided port
                PORT = varargin{1};
            end

            % Get Current Dir of Launcher.m file (Programs)
            currentFile = mfilename('fullpath');
            [currentDir, ~, ~] = fileparts(currentFile);

            % Get Parent Dir (src)
            [parentDir, ~, ~] = fileparts(currentDir);

            dynamixel_lib_path = fullfile(parentDir, 'DynamixelLib\c\');

            obj.realRobot = RealRobot(dynamixel_lib_path,PORT);

            obj.realRobot.torqueEnable;

            fprintf("Launcher: Torque Enabled. \n");

        end

        function disconnect(obj)
            obj.stopCurrentProgram;
            if ~isempty(obj.realRobot)
                delete(obj.realRobot)
                obj.realRobot = [];
            end

        end

        function instantiateProgram(obj, programName)
            % Check if the programName is in the available programs
            if ismember(programName, obj.programNames)

                % Stop the current program
                obj.stopCurrentProgram;

                % Instantiate and the program using its name
                % Pass a reference to the launcher instance and variable
                % arguments
                try
                    obj.currentProgramInstance = feval(programName, obj);
                    fprintf('Insantiated Program: %s \n', obj.currentProgramInstance.name);
                catch ME
                    fprintf('Launcher: Failed to instantiate %s. Error: %s. '\n', programName, ME.message );
                end        

            else
                fprintf('Launcher: Program %s not found in available programs. \n', programName);
            end
        end

        function stopCurrentProgram(obj)
            if ~isempty(obj.currentProgramInstance) && obj.currentProgramInstance.is_running
                obj.currentProgramInstance.stop;
            end
        end
        
    end

    methods (Hidden)
        function notifyProgramStopped(obj, program)
            % Notification when a program is stopped, this gets called by
            % the program itself
            fprintf('Launcher: %s has stopped. \n', program.name);

            % Halting
            obj.realRobot.setJointVelocities([0;0;0;0])
            fprintf('Halting. \n');

        end

        function notifyProgramCrashed(obj, program)
            % Notification when a program is stopped, this gets called by
            % the program itself
            warning('Launcher: Program %s has crashed. \n', program.name);

            % Halting
            obj.realRobot.setJointVelocities([0;0;0;0])
            fprintf('Halting. \n');

            obj.currentProgramInstance = [];
        end
    end


    methods (Static)

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
