classdef (Abstract) AbstractProgram < handle

    properties (Abstract, Constant)
        name % Name of the program
    end
    
    properties
        launcher  % Reference to the Launcher object
        is_running;
    end

    methods

        function obj = AbstractProgram(launcherObj, varargin)
            obj.launcher = launcherObj;
            obj.is_running = false;
        end

        % Abstract methods
        execute(obj);  % Method to execute the program

    end

    methods (Hidden)

        % Common method for updating and drawing the robot configuration
        function updateConfig(obj)
            % Update and plot logic
            obj.launcher.virtualRobot.setQ(obj.launcher.realRobot.getQ);
            obj.launcher.virtualRobot.draw;

        end

        % Notify the Launcher that the program stopped
        function stop(obj)
            obj.launcher.notifyProgramStopped(obj);  % Notify Launcher about the stop
            obj.is_running = false;
        end

        % Cleanup actions (Strg + C during Execution or Error during execution)
        function cleanup(obj)
            % If the programs state indicates that it should be running but
            % it closed abnormaly --> crash
            if obj.is_running
                obj.launcher.notifyProgramCrashed(obj);
            end
        end
    end
end
