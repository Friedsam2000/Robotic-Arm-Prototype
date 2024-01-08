classdef Set_Joints < AbstractProgram

    properties (Constant)
        default_Kp = 2;
        default_precision_deg = 0.5;
    end


    methods

        function start(obj, q_desired, varargin)

            % Setup a cleanup function that gets called when Strg + C
            % during loop or program crashes
            cleanupObj = onCleanup(@() obj.stop());
            % Set Status
            obj.launcher.status = 'executing';
            % Initial drawing
            obj.updateConfigAndPlot;

            % Parse optional arguments
            p = inputParser;
            addOptional(p, 'Kp', obj.default_Kp);
            addOptional(p, 'precision_deg', obj.default_precision_deg);
            parse(p, varargin{:});
            Kp = p.Results.Kp;
            precision_deg = p.Results.precision_deg;
            precision_rad = deg2rad(precision_deg);

            % P-Control Loop for Joint Positions
            while 1

                % Update virtual robot and plot
                obj.updateConfig;
                q = obj.launcher.virtualRobot.getQ;

                % Calculate remaining errors
                currentError = q_desired - q;

                % Set velocities
                PID_velocities = Kp * currentError;
                obj.launcher.realRobot.setJointVelocities(PID_velocities);

                % Check if joints converged
                if all(abs(currentError) <= precision_rad)
                    break;
                end

                % Maybe unecessary
                pause(0.01);  % Pause to allow MATLAB to process other events
            end
            obj.stop;
        end
    end
end