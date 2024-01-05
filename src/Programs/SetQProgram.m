classdef SetQProgram < Program
    properties
        q_desired
        Kp = 1;
        precision = 0.5;  % Default precision in degrees
    end

    methods
        function obj = SetQProgram(launcherObj, q_desired, varargin)
            % Call superclass constructor
            obj@Program(launcherObj, varargin{:});

            obj.q_desired = q_desired;

            % Parse optional arguments
            p = inputParser;
            addOptional(p, 'Kp', obj.Kp, @isnumeric);
            addOptional(p, 'precision', obj.precision, @isnumeric);
            parse(p, varargin{:});

            obj.Kp = p.Results.Kp;
            obj.precision = deg2rad(p.Results.precision); % Convert to radians
        end

        function execute(obj)

            % Setup a cleanup function that gets called when Strg + C
            % during loop
            cleanupObj = onCleanup(@() obj.cleanup());
            obj.launcher.realRobot.torqueEnable;

            % Initial drawing
            q = obj.updateConfig;
            obj.launcher.virtualRobot.workspace.draw;
            

            % P-Control Loop for Joint Positions
            while 1
                % Update virtual robot and plot
                q = obj.updateConfig;

                % Calculate remaining errors
                currentError = obj.q_desired - q;

                % Set velocities
                PID_velocities = obj.Kp * currentError;
                obj.launcher.realRobot.setJointVelocities(PID_velocities);

                % Check if joints converged
                if all(abs(currentError) <= obj.precision)
                    fprintf("Program: All joints converged \n");
                    break;
                end

                pause(0.01);  % Pause to allow MATLAB to process other events
            end
            obj.stop;
        end

        function stop(obj)
            obj.launcher.realRobot.setJointVelocities([0; 0; 0; 0]);
            obj.launcher.notifyProgramStopped(obj);  % Notify Launcher about the stop
        end

        function cleanup(obj)
            obj.stop
        end
    end
end