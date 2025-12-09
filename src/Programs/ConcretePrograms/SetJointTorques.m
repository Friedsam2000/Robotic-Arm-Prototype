classdef SetJointTorques < Program
    
    properties(Constant)
        runtime = 10; %seconds
    end

    properties
        timer = 0;
        desired_torque = [];
    end

    methods

        function setup(obj, varargin)

            p = inputParser;
            addRequired(p, 'desired_torque', @(x) isvector(x) && length(x) == 4 && all(isnumeric(x)) && iscolumn(x));
            addParameter(p, 'Kp', 1);
            parse(p, varargin{:});
            obj.desired_torque = p.Results.desired_torque;
            obj.timer = tic;
            r = obj.launcher.realRobot;
            r.setRobotTorque(0)
            r.setRobotOperationMode(0)
            r.setRobotTorque(1)
        end

        function loop(obj)

            obj.stopCondition = obj.launcher.virtualRobot.checkJointLimits;
            
            r = obj.launcher.realRobot;
            r.setJointTorques(obj.desired_torque)

            % if toc(obj.timer) > obj.runtime
            %     r.setJointTorques([0 0 0 0])
            %     r.setRobotTorque(0)
            %     r.setRobotOperationMode(1)
            %     r.setRobotTorque(1)
            % 
            %     obj.stopCondition = true;
            % end

        end
    end

    methods (Static)
        function argsInfo = getArgumentsInfo()
            argsInfo.name = 'Joint Torques [Nm]';
            argsInfo.placeholder = 't1; t2; t3; t4';
            argsInfo.validationPattern = '^(-?\d+(\.\d+)?[;] *){3}-?\d+(\.\d+)?$'; % Pattern for validation
        end
    end
end