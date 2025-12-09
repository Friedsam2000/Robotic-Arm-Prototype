classdef SpringJoints < Program


    properties
        kp = 1;
        kd = 1;
        goal_pos = [];
        time = 0;
    end

    methods

        function setup(obj, varargin)

            p = inputParser;
            addRequired(p, 'spring');
            addParameter(p, 'Kp', 1);
            parse(p, varargin{:});
            %obj.K_gain = p.Results.Kp;
            obj.goal_pos = obj.launcher.virtualRobot.getJointAngles;

            r = obj.launcher.realRobot;
            r.setRobotTorque(0)
            r.setRobotOperationMode(0)
            r.setRobotTorque(1)

            obj.time = tic;
        end

        function loop(obj)

            obj.stopCondition = obj.launcher.virtualRobot.checkSingularity;

            r = obj.launcher.realRobot;
            v = obj.launcher.virtualRobot;

            % PD-controller

            delta_q = v.getJointAngles - obj.goal_pos;
            %normalizing the error:
            delta_q = delta_q./(v.JOINT_ANGLE_LIMITS(:,2).^2);

            q_dot = r.getJointVelocities;

            kp_vec = [10;10;200;40];
            kd_vec = [0.3;0.3;0.1;0.16];

            desired_t = - kp_vec .* delta_q - kd_vec .* q_dot;

            r.setJointTorques(desired_t)

            %fprintf("avg loop frequency: %.2f Hz\n", obj.launcher.getAvgLoopFreq)
        end
    end

    methods (Static)
        function argsInfo = getArgumentsInfo()
            argsInfo.name = '';
            argsInfo.placeholder = '';
            argsInfo.validationPattern = '.*';
        end
    end
end