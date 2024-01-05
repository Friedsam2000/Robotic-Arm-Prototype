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

        % Common method for updating and drawing the robot configuration
        function q = updateConfig(obj)
            % Update and plot logic
            q = obj.launcher.realRobot.getQ;
            obj.launcher.virtualRobot.setQ(q);
            obj.launcher.virtualRobot.draw;
            obj.launcher.virtualRobot.frames(end).draw;
        end

        function stop(obj)
            % Stop method implementation
            obj.launcher.realRobot.setJointVelocities([0; 0; 0; 0]);
            obj.launcher.notifyProgramStopped(obj);  % Notify Launcher about the stop
        end

        function cleanup(obj)
            % Cleanup actions
            obj.stop();
        end
    end
end
