classdef (Abstract) Program < handle

    properties
        launcher = [];  % Reference to the Launcher object
        stopCondition = []; % True when the Program should stop
    end

    methods
        
        % Abstract methods, to be defined in ConcretePrograms
        setup(obj,varargin);
        loop(obj);

        %% Constructor
        function obj = Program(launcher)
            % A reference to the launcher object that instantiated the program
            obj.launcher = launcher;
        end

        %% Destructor
        function delete(obj)
            % Execute the callback in the launcher before deletion
            obj.launcher.programTerminationCallback;
        end

        %% Start
        function start(obj, varargin)
            obj.stopCondition = false;
            try
                obj.setup(varargin{:});
                while ~obj.stopCondition
                    obj.loop;
                    obj.launcher.syncJointsAndPlot;
                end
            catch ME
                % Output the error message with the program name
                % fprintf('Error in %s: %s\n', class(obj), getReport(ME, 'basic'));
            end
            delete(obj);
        end
    end
end
