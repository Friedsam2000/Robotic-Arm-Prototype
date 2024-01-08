classdef Plotting < AbstractProgram
    
    properties (Constant)
         targetUpdatePeriod = 0.1;
    end
    

    methods
        
        function start(obj)
            start(obj.timerObj);
        end
        
        function obj = Plotting(launcherObj, varargin)
            % Call the constructor of the superclass
            obj@AbstractProgram(launcherObj, varargin);

            % Create and start timer, delete object if timer crashes / stops
            obj.timerObj = timer('ExecutionMode', 'fixedRate', 'Period', obj.targetUpdatePeriod, ...
                                 'BusyMode', 'drop', 'TimerFcn', @(~,~) obj.timerCallback, 'ErrorFcn', @(~,~) delete(obj));
        end

        function timerCallback(obj)
            % Update Config and Plot
            obj.updateConfigAndPlot;
            % Update Singularity Status
            if obj.launcher.virtualRobot.checkSingularity
                obj.launcher.status = 'singularity';
            else
                obj.launcher.status = 'ready';
            end
        end
        
    end
end
