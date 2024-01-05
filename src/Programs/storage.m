
    % 
    %             %% Program Body

                obj.launcher.realRobot.torqueEnable;

    %             % Go to initial position
                obj.launcher.realRobot.setJointVelocities([0.2;0.2;0.2;0.2]);
                pause(1);
    % 
    %             % Initialize the controller
                controller = NullspaceController(obj.launcher.virtualRobot);
    % 
    %             % Initialize the planner
                planner = PathPlanner2D(obj.launcher.virtualRobot, trajectory_height);
    % 
    %             % Prompt the user to draw a path
                planner.userInputPath;
    % 
    %             % Initialize the trajectory generator
                trajectoryGenerator = TrajectoryGenerator(planner.getWaypointList, trajectory_time);
                [x_d, v_d, t] = trajectoryGenerator.getTrajectory;
    % 
    %             % Plot the desired trajectory
                obj.virtualRobot.draw
                trajectoryGenerator.draw(obj.virtualRobot.fig)

    %             % Plot the workspace
                obj.virtualRobot.workspace.draw;
    % 
    %             %% Control Loop
    % 
    %             % Variables for time tracking
    %             loopBeginTime = tic;  % Start the timer
    % 
    %             while true
    %                 % Simulation
    %                 q = obj.realRobot.getQ;
    %                 obj.virtualRobot.setQ(q);
    % 
    %                 % Calculate elapsed time and the time increment (dt)
    %                 elapsedRealTime = toc(loopBeginTime);
    % 
    %                 % Find the index in the trajectory that corresponds to the elapsed time
    %                 [~, index] = min(abs(t - elapsedRealTime));
    % 
    %                 % Break the loop if the end of the trajectory is reached
    %                 if index >= length(t)
    %                     obj.setQ([0;0;0;0],true);
    %                     obj.realRobot.torqueDisable;
    %                     break;
    %                 end
    % 
    %                 % Get the desired position and velocity for the current timestep
    %                 current_x_d = x_d(:, index);
    %                 current_v_d = v_d(:, index);
    % 
    %                 % Compute the desired joint velocity
    %                 q_dot = controller.computeDesiredJointVelocity(current_x_d, NaN, current_v_d);
    % 
    %                 % Set joint speed of real robot
    %                 obj.realRobot.setJointVelocities(q_dot);
    % 
    %                 % Update and draw the end-effector trajectory and the robot
    %                 obj.virtualRobot.updateTrajectory;
    %                 obj.virtualRobot.draw;
    % 
    %             end
    % 
    % 
    %             %%
    %             if ~bypass_busy
    %                 obj.is_busy = false;
    %             end
    %         end
    %     end
    % 
    % end
    % 
    % 
    %     %% END OF PROGRAMS
    % 
    % 
    % methods (Access = private)
    % 
    %     function setupUpdateTimer(obj)
    %         % Setup and start the update timer
    %         obj.updateTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.1, ...
    %                                 'BusyMode', 'drop', ...
    %                                 'TimerFcn', @(~,~) obj.updateVirtualRobotAndPlot());
    %     end
    % 
    %     function updateVirtualRobotAndPlot(obj)
    %         % Timer callback function for updating and plotting
    %         if ~obj.is_busy
    % 
    %             % disp("Timer update")
    %             obj.is_busy = true;
    %             q = obj.realRobot.getQ;
    %             obj.virtualRobot.setQ(q);
    %             obj.virtualRobot.draw;
    %             obj.virtualRobot.frames(end).draw;
    %             obj.is_busy = false;
    %         end
    %     end

    end


