classdef (Abstract) Program < handle
    properties
        launcher  % Reference to the Launcher object
    end

    methods
        function obj = Program(launcherObj, varargin)
            obj.launcher = launcherObj;
            % Additional constructor logic can be added here if needed
        end

        % Abstract methods
        execute(obj);  % Method to execute the program
        stop(obj);     % Method to stop the program
    end
end
