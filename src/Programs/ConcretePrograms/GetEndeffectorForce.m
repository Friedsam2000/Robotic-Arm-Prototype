classdef GetEndeffectorForce < Program

    methods

        function setup(obj, varargin)

            p = inputParser;
            addRequired(p, 'F');
            addParameter(p, 'Kp', 1);
            parse(p, varargin{:});

            r = obj.launcher.realRobot;
            r.setJointVelocities([0;0;0;0])
        end

        function loop(obj)

            obj.stopCondition = obj.launcher.virtualRobot.checkSingularity;

            r = obj.launcher.realRobot;
            v = obj.launcher.virtualRobot;
            
            J = v.getJacobian();

            t = r.getJointTorques();

            F = pinv(J') * t *1000;

            fprintf("TCP force = [%f, %f, %f]\n",F)
            
            
        end
    end

    methods (Static)
        function argsInfo = getArgumentsInfo()
            argsInfo.name = '';
            argsInfo.placeholder = '';
            % Pattern to match three doubles with semicolons as separators, allowing negative values and optional spaces around separators
            argsInfo.validationPattern = '.*';
        end
    end
end