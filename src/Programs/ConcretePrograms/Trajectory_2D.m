classdef Trajectory_2D < AbstractProgram

    properties (Constant)
        default_trajectoryTime = 15; % [s]
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
            g_r_EE = programObj.launcher.virtualRobot.getEndeffectorPos;
            currentHeight = g_r_EE(3);

            % Parse optional arguments
            p = inputParser;
            addOptional(p, 'trajectoryHeight', currentHeight); % Default height is current height
            addOptional(p, 'trajectoryTime', programObj.default_trajectoryTime);
            parse(p, varargin{:});
            trajectoryHeight = p.Results.trajectoryHeight;
            trajectoryTime = p.Results.trajectoryTime;

            % Controller and Planner
            controller = NullspaceController(programObj.launcher.virtualRobot);
            planner = PathPlanner2D(programObj.launcher.virtualRobot, trajectoryHeight);
            planner.userInputPath; % User inputs path

            % Trajectory Generator
            trajectoryGenerator = TrajectoryGenerator(planner.getWaypointList, trajectoryTime);
            [x_d, v_d, t] = trajectoryGenerator.getTrajectory;

            % Draw the desired trajectory
            trajectoryGenerator.draw(programObj.launcher.virtualRobot.fig);

            loopBeginTime = tic;
            % Control Loop executes while the program is not deleted, the
            % robot is not in a singularity configuration or a break
            % condition is met (e.g. position reached)
            while ~programObj.launcher.singularityWarning
               
                % Time calculations
                elapsedRealTime = toc(loopBeginTime);
                [~, index] = min(abs(t - elapsedRealTime));
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
