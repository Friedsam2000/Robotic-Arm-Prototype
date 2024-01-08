classdef Plotting < AbstractProgram
    
    properties (Constant)
         name = "Plotting";
    end
    
    properties (Access = private)
        timerObj  % Timer object
    end

    methods
        function execute(obj)
            obj.is_running = true;
            obj.createAndStartTimer;
        end

    end

    methods (Hidden)

        function stop(obj)
            % Notify the Launcher that the program stopped
            obj.stopAndDeleteTimer;
            obj.launcher.notifyProgramStopped(obj);  % Notify Launcher about the stop
            obj.is_running = false;
        end
        
        function stopAndDeleteTimer(obj)
            if ~isempty(obj.timerObj)
                if strcmp(obj.timerObj.Running, 'on')
                    stop(obj.timerObj);
                end
                delete(obj.timerObj);
            end
        end

        function createAndStartTimer(obj)
            % Delete timer if there is one
            obj.stopAndDeleteTimer;
            % Create and start timer
            obj.timerObj = timer('ExecutionMode', 'fixedRate', 'Period', 0.1, ...
                                 'BusyMode', 'drop', 'TimerFcn', @(~,~) obj.updateConfig);
            start(obj.timerObj);
        end
    end
end
