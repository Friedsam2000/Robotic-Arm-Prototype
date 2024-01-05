classdef Launcher < handle
    properties (SetAccess=private)
        virtualRobot = [];
        realRobot = [];
        currentProgram
    end
    
    methods (Static)
        function obj = getInstance()
            persistent uniqueInstance
            if isempty(uniqueInstance) || ~isvalid(uniqueInstance)
                uniqueInstance = Launcher();
            end
            obj = uniqueInstance;
        end
    end

    methods (Access=private)
        function obj = Launcher()
            % Constructor
            obj.initPath();
            obj.virtualRobot = VirtualRobot;

        end
    end

    methods

        function delete(obj)
             % Destructor
            obj.disconnect;
            if ~isempty(obj.currentProgram)
                obj.currentProgram.stop();
            end
            delete(obj.realRobot);
            delete(obj.virtualRobot);
        end

        function connect(obj,varargin)

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

            currentFile = mfilename('fullpath');

            [currentDir, ~, ~] = fileparts(currentFile);

            dynamixel_lib_path = fullfile(currentDir, 'src\DynamixelLib\c\');

            obj.realRobot = RealRobot(dynamixel_lib_path,PORT);
            obj.realRobot.setZeroPositionToCurrentPosition;
            disp("Launcher: Zero Position Set")
            obj.realRobot.torqueEnable;
            disp("Launcher: Torque Enabled")

        end

        function disconnect(obj)
            obj.stopCurrentProgram;
            if ~isempty(obj.realRobot)
                delete(obj.realRobot)
            end

        end

        function setProgram(obj, programClassName, varargin)
            % Set the current program based on the class name and arguments
            if exist(programClassName, 'class') == 8 % Check if the class exists
                if ~isempty(obj.currentProgram) && isa(obj.currentProgram, programClassName)
                    % Check if the current program is of the same class as the new program
                    disp(['Launcher: ', programClassName, ' is already running.']);
                    return;
                end

                newProgramInstance = feval(programClassName, obj, varargin{:});
                if isa(newProgramInstance, 'Program')
                    if ~isempty(obj.currentProgram)
                        obj.currentProgram.stop();
                    end
                    obj.currentProgram = newProgramInstance;
                    obj.currentProgram.execute();
                else
                    error(['Launcher: "', programClassName, '" is not a valid Program class.']);
                end
            else
                error(['Launcher: Program class "', programClassName, '" not found.']);
            end
        end

        function stopCurrentProgram(obj)
            if ~isempty(obj.currentProgram)
                obj.currentProgram.stop();
            else
                disp("Launcher: No program running")
            end
        end
    
    end


    methods (Hidden)
        function notifyProgramStopped(obj, program)
            % Notification when a program is stopped
            if isequal(obj.currentProgram, program)
                disp(['Launcher: ', class(program), ' has stopped.']);
                obj.currentProgram = [];
            end
        end
    end


    methods (Static)

        function initPath(~)
            % Initialize the MATLAB path

            clc;
            close all;
            currentFile = mfilename('fullpath');

            [currentDir, ~, ~] = fileparts(currentFile);
            addpath(fullfile(currentDir, 'src\VirtualRobot'));
            addpath(fullfile(currentDir, 'src\RealRobot'));
            addpath(fullfile(currentDir, 'src\Planner'));
            addpath(fullfile(currentDir, 'src\Controller'));
            addpath(fullfile(currentDir, 'src\Programs'));

        end           

    end

end
