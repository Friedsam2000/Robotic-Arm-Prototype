classdef UpdateAndPlotProgram < Program
    properties
        timerObj  % Timer object
    end

    methods
        function obj = UpdateAndPlotProgram(launcherObj)
            obj@Program(launcherObj);
            % Initialize the timer object
            obj.initializeTimer();
        end

        function execute(obj)
            if ~isvalid(obj.timerObj)
                % Reinitialize the timer if it's not valid
                obj.initializeTimer();
            end
            obj.launcher.virtualRobot.draw;
            obj.launcher.virtualRobot.workspace.draw;
            start(obj.timerObj);
        end

        function stop(obj)
            if isvalid(obj.timerObj) && strcmp(obj.timerObj.Running, 'on')
                stop(obj.timerObj);
                delete(obj.timerObj);
            end
            obj.launcher.notifyProgramStopped(obj); % Notify Launcher about the stop
        end

        function initializeTimer(obj)
            % Method to initialize the timer object
            obj.timerObj = timer('ExecutionMode', 'fixedRate', 'Period', 0.1, ...
                                 'BusyMode', 'drop', 'TimerFcn', @(~,~) obj.safeUpdate());
        end

        function safeUpdate(obj)
            try
                 q = obj.updateConfig(); % Try to run the update method
            catch ME
                disp(['Program: Error encountered in Update and Plot Program: ', ME.message]);
                obj.stop(); % Stop the program
            end
        end
    end
end
