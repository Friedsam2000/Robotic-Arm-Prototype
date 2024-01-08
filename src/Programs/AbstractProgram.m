classdef (Abstract) AbstractProgram < handle

    properties
        launcher  % Reference to the Launcher object
        timerObj % Reference to a timer obj if the program uses one
    end

    methods
        % Abstract methods have to be defined in concrete program
        start(obj);  % Method to start
    end

    methods (Hidden)

        function obj = AbstractProgram(launcherObj, varargin)
            obj.launcher = launcherObj;
        end

        function delete(obj)
            if isvalid(obj.launcher)
                obj.launcher.status = 'ready';
                obj.launcher.currentProgramInstance = [];
            end
            % Delete timer if there is one
            if isvalid(obj.timerObj)
                if strcmp(obj.timerObj.Running, 'on')
                    stop(obj.timerObj);
                end
                delete(obj.timerObj);
            end
        end

        function updateConfigAndPlot(obj)
            % Check if figure is still valid
            if isempty(obj.launcher.virtualRobot.fig) || ~isvalid(obj.launcher.virtualRobot.fig)
                obj.launcher.virtualRobot.initRobotPlot;
            end

            % Common method to update the virtual robots configuration and update the plot
            obj.launcher.virtualRobot.setQ(obj.launcher.realRobot.getQ);
            obj.launcher.virtualRobot.updateRobotPlot;
        end
    end
end
