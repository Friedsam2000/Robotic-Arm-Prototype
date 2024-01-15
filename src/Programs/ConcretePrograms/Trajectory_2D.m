classdef Trajectory_2D < AbstractProgram

    properties (Constant)
        default_Kp = 2;
    end

    methods
        function start(programObj,varargin)
            
            % Update Config and Plot
            programObj.launcher.updateConfigAndPlot;

            if programObj.launcher.singularityWarning
                delete(programObj);
                return
            end
            
            % Get Current Height
            g_r_EE = programObj.launcher.virtualRobot.forwardKinematics;
            currentHeight = g_r_EE(3);

            % Parse optional arguments
            p = inputParser;
            addRequired(p, 'trajectoryTime', @(x) isscalar(x) && isnumeric(x));
            addOptional(p, 'Kp', programObj.default_Kp); % Default Kp value
            addOptional(p, 'trajectoryHeight', currentHeight); % Default height is current height

            parse(p, varargin{:});
            trajectoryTime = p.Results.trajectoryTime;
            Kp_final = p.Results.Kp; % Final Kp value
            trajectoryHeight = p.Results.trajectoryHeight;


            % Initialize Kp ramp
            Kp = 0;
            ramp_duration = 1; % Ramp duration in seconds

            % Controller and Planner
            controller = NullspaceController(programObj.launcher.virtualRobot);
            planner = PathPlanner2D(programObj.launcher.virtualRobot, trajectoryHeight);
            planner.userInputPath; % User inputs path

            % Trajectory Generator
            trajectoryGenerator = TrajectoryGenerator(planner.getWaypointList, trajectoryTime);
            [x_d, v_d, t] = trajectoryGenerator.getTrajectory;

            % Draw the desired trajectory
            trajectoryGenerator.draw(programObj.launcher.virtualRobot.fig);

            % Control Loop executes while the program is not deleted, the
            % robot is not in a singularity configuration or a break
            % condition is met (e.g. position reached)
            start_time = tic; % Start timer
            while ~programObj.launcher.singularityWarning
               
                % Calculate elapsed time
                elapsed_time = toc(start_time);

                % Ramp Kp
                if elapsed_time < ramp_duration
                    Kp = Kp_final * (elapsed_time / ramp_duration);
                else
                    Kp = Kp_final;
                end

                % Update controller Kp
                controller.Kp = Kp;

                [~, index] = min(abs(t - elapsed_time));
                if index >= length(t)
                    fprintf("Program %s: Trajectory time elapsed. \n", class(programObj))
                    break; % Exit loop at the end of the trajectory
                end

                % Set velocities
                q_dot = controller.computeDesiredJointVelocity(x_d(:, index), NaN, v_d(:, index));
                programObj.launcher.realRobot.setJointVelocities(q_dot);
    
                % Update Config and Plot
                programObj.launcher.updateConfigAndPlot;

            end
            % Delete the program upon finish
            delete(programObj)
        end
    end
end
