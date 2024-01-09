classdef (Abstract) AbstractProgram < handle

    properties
        launcher  % Reference to the Launcher object
        timerObj = []; % Reference to a timer obj if the program uses one
        onDeleteCallback = []; % Function handle for delete callback
    end

    methods

        % Abstract definition of
        concreteProgram(obj,varargin);

        function start(obj, varargin)  % Method to start
            try
                obj.concreteProgram(varargin{:});
            catch
                delete(obj)
            end
        end

        % Constructor
        function obj = AbstractProgram(launcherObj, onDeleteCallback)
            obj.launcher = launcherObj;
            obj.onDeleteCallback = onDeleteCallback;
        end

        % Delete method
        function delete(obj)
            if ~isempty(obj.launcher) && isvalid(obj.launcher)
                fprintf('Program %s: Halting \n', class(obj));
                obj.launcher.realRobot.setJointVelocities([0;0;0;0])
            end

            % Execute the delete callback if it's a non-empty function handle
            if ~isempty(obj.onDeleteCallback) && isa(obj.onDeleteCallback, 'function_handle')
                obj.onDeleteCallback(obj);
            end
            % Delete timer if there is one
            if ~isempty(obj.timerObj) && isvalid(obj.timerObj)
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

            % Update Singularity Status
            obj.launcher.singularityWarning = obj.launcher.virtualRobot.checkSingularity;
        end
    end
end
