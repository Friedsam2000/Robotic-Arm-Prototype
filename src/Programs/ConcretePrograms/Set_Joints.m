classdef Set_Joints < AbstractProgram

    properties (Constant)
        name = "Set_Joints";
    end


    methods

        function execute(obj, q_desired, varargin)

            % Setup a cleanup function that gets called when Strg + C
            % during loop or program crashes
            cleanupObj = onCleanup(@() obj.cleanup());

            % Initial drawing
            obj.updateConfig;

            % Parse optional arguments
            % p = inputParser;
            % addOptional(p, 'trajectoryHeight', currentHeight); % Default height is current height
            % parse(p, varargin{:});
            % trajectoryHeight = p.Results.trajectoryHeight;

            % P-Control Loop for Joint Positions
            while 1
                % Update virtual robot and plot
                q = obj.launcher.realRobot.getQ;

                % Calculate remaining errors
                currentError = q_desired - q;

                % Set velocities
                PID_velocities = Kp * currentError;
                obj.launcher.realRobot.setJointVelocities(PID_velocities);

                % Check if joints converged
                if all(abs(currentError) <= obj.precision)
                    printf("Program: All joints converged \n");
                    break;
                end

                pause(0.01);  % Pause to allow MATLAB to process other events
            end
            obj.stop;
        end
    end
end