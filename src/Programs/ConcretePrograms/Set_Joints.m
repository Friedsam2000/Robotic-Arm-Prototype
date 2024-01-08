classdef Set_Joints < AbstractProgram

    properties (Constant)
        name = "Set_Joints";
    end


    methods

        function execute(obj, q_desired, varargin)

            % Setup a cleanup function that gets called when Strg + C
            % during loop or program crashes
            cleanupObj = onCleanup(@() obj.cleanup());
            obj.is_running = true;

            % Initial drawing
            obj.updateConfig;

            % Parse optional arguments
            p = inputParser;
            addOptional(p, 'Kp', 2); % Default Kp is 1.5
            addOptional(p, 'precision_deg', 0.5); % Default precision is 0.5 deg

            parse(p, varargin{:});
            Kp = p.Results.Kp;
            precision_deg = p.Results.precision_deg;

            precision_rad = deg2rad(precision_deg);

            % P-Control Loop for Joint Positions
            while 1
                % Update virtual robot and plot
                obj.updateConfig;
                q = obj.launcher.realRobot.getQ;

                % Calculate remaining errors
                currentError = q_desired - q;

                % Set velocities
                PID_velocities = Kp * currentError;
                obj.launcher.realRobot.setJointVelocities(PID_velocities);

                % Check if joints converged
                if all(abs(currentError) <= precision_rad)
                    fprintf("Program: All joints converged \n");
                    break;
                end

                pause(0.01);  % Pause to allow MATLAB to process other events
            end
            obj.stop;
        end
    end
end