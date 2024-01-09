classdef Plotting < AbstractProgram
    
    properties (Constant)
         targetUpdatePeriod = 0.1;
    end

    methods
        
        function concreteProgram(obj, varargin)
            start(obj.timerObj);
        end
        
        function obj = Plotting(launcherObj, onDeleteCallback)
            % Call the constructor of the superclass
            obj@AbstractProgram(launcherObj, onDeleteCallback);

            % Create and start timer, delete object if timer crashes/stops
            obj.timerObj = timer('ExecutionMode', 'fixedRate', 'Period', obj.targetUpdatePeriod, ...
                                 'BusyMode', 'drop', 'TimerFcn', @(~,~) obj.updateConfigAndPlot, 'ErrorFcn', @(~,~) delete(obj));
        end
        
    end
end
