classdef (Abstract) AbstractProgram < handle

    properties (Abstract, Constant)
        name % Name of the program
    end
    
    properties
        launcher  % Reference to the Launcher object
    end

    methods
        function obj = AbstractProgram(launcherObj, varargin)
            obj.launcher = launcherObj;
        end

        % Abstract methods
        execute(obj);  % Method to execute the program

        % Common method for updating and drawing the robot configuration
        function updateConfig(obj)
            % Update and plot logic
            obj.launcher.realRobot.getQ;
            obj.launcher.virtualRobot.setQ(q);
            obj.launcher.virtualRobot.draw;
            obj.launcher.virtualRobot.frames(end).draw;
            obj.launcher.virtualRobot.frames(1).draw;
            obj.launcher.virtualRobot.workspace.draw;

        end

        % Notify the Launcher that the program stopped
        function stop(obj)
            obj.launcher.notifyProgramStopped(obj);  % Notify Launcher about the stop
        end

        % Cleanup actions (Strg + C during Execution or Error during execution)
        function cleanup(obj)
            obj.launcher.notifyProgramCrashed(obj);
        end
    end
end
