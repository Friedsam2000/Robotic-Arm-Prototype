classdef (Abstract) AbstractProgram < handle

    properties
        launcher  % Reference to the Launcher object
    end

    methods

        % Abstract definition of the starting method of the concrete program
        start(programObj, varargin);

        % Constructor
        function programObj = AbstractProgram(launcher)
            % A reference to the launcher object that instantiated the program
            programObj.launcher = launcher;
        end

        % Delete method
        function delete(programObj)
            % Execute the callback in the launcher before deleting itself
            programObj.launcher.programDeleteCallback          
        end

    end
end
