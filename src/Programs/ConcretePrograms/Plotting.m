classdef Plotting < AbstractProgram
    
    properties (Constant)
         name = "Plotting";
    end
    
    properties (Access = private)
        timerObj  % Timer object
    end

    methods
        function execute(obj)
            % Setup a cleanup function that gets called when Strg + C
            % during loop or program crashes
            cleanupObj = onCleanup(@() obj.cleanup());

            obj.createAndStartTimer;
        end

    end

    methods (Access=private)

        function stopAndDeleteTimer(obj)
            if ~isempty(obj.timerObj) && isa(obj.timerObj, 'timer')
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
