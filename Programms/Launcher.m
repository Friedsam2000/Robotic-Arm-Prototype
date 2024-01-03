classdef Launcher < handle
    properties (Access = private)

        virtualRobot
        realRobot

        updateTimer

        % Flag to signal that the robot is busy
        is_busy = false; 
        
        started = false;
    end
    
    methods
        function obj = Launcher()
            % Constructor
            obj.initPath();
            obj.virtualRobot = VirtualRobot;
            obj.realRobot = RealRobot;
            obj.setupUpdateTimer();
        end

        function delete(obj)
            obj.stop;

            % Destructor to clean up resources
            % Find all running timers
            allTimers = timerfind;
            
            % Stop all found timers
            if ~isempty(allTimers)
                stop(allTimers);
                delete(allTimers);
            end            
            disp('Timer stopped and deleted.');

            delete(obj.realRobot);
            delete(obj.virtualRobot);
        end

        function start(obj)

            if obj.started == true
                disp("Already started, run stop first and return to zero position")
                return
            end

            % Upon starting the zero position gets set

            obj.realRobot.setZeroPositionToCurrentPosition;
            obj.realRobot.torqueEnableDisable(0);

            obj.virtualRobot.draw;
            obj.virtualRobot.workspace.draw;
            start(obj.updateTimer);
            obj.started = 1;
        end

        function stop(obj)
            
            if obj.started == false
                disp("Already stopped, return to zero position and run start")
                return
            end


            if isprop(obj, 'updateTimer') && ~isempty(obj.updateTimer) && isvalid(obj.updateTimer)
                % The timer exists and is a valid object
                if strcmp(obj.updateTimer.Running, 'on')
                    % The timer is running, so stop it
                    stop(obj.updateTimer);
                end
            end

            obj.started = 0;

            obj.is_busy = false;
            obj.realRobot.setJointVelocities([0; 0; 0; 0]);
            obj.realRobot.torqueEnableDisable(0);

            close all

        end

        %% Programs

        function setQ(obj,q_desired,varargin)
            if ~obj.started
                disp("Run start method first!");
                return
            end

            % Default value for bypass_busy
            bypass_busy = false;
        
            % Check if bypass_busy is provided in varargin
            if ~isempty(varargin)
                bypass_busy = varargin{1};
            end

            if (~obj.is_busy || bypass_busy)
                
                obj.is_busy = true;
    
                %% Program Body
    
                obj.realRobot.torqueEnableDisable(1);
    
                % Use a default precision of 0.1 °
                precision_deg = 0.1;
                precision = deg2rad(precision_deg);
    
                % Initialize errors and gains for the PID controller. This
                % controller tries to move the servos to their stored zero
                % position by setting their servo.
                integralError = [0; 0; 0; 0];
                prevError = [0; 0; 0; 0];
                P_Gain = 1;  % Proportional gain
                I_Gain = 0;  % Integral gain
                D_Gain = 0;  % Derivative gain
    
                jointsConverged = [0;0;0;0];
        
                while 1
                    % Update virtual robot and plot
                    q = obj.realRobot.getQ;
                    obj.virtualRobot.setQ(q);
                    obj.virtualRobot.draw;
                    obj.virtualRobot.frames(end).draw;
       
                    % Calculate remaining errors
                    currentError = q_desired-q; % As they should become 0
                    integralError = integralError + currentError;
                    derivativeError = currentError - prevError;
                    prevError = currentError;
                   
                    %Set velocites
                    PID_velocities = P_Gain * currentError + I_Gain * integralError + D_Gain * derivativeError;
    
                    obj.realRobot.setJointVelocities(PID_velocities);
    
                    % Check if joints converged
                    for i = 1:4
                        if abs(currentError(i)) <= precision
                            jointsConverged(i) = 1;
                        end
                    end
                    
                    % If all joints converged disable the torque and return
                    if sum(jointsConverged) == 4
                        obj.realRobot.setJointVelocities([0;0;0;0])
                        fprintf("All joints converged \n")
                        break
                    end
                end
                %%
                if ~bypass_busy
                    obj.is_busy = false;
                end
            end
        end

        function goToPos(obj, x_desired, varargin)
            if ~obj.started
                disp("Run start method first!");
                return
            end

             % Default value for bypass_busy
            bypass_busy = false;
        
            % Check if bypass_busy is provided in varargin
            if ~isempty(varargin)
                bypass_busy = varargin{1};
            end

            if (~obj.is_busy || bypass_busy)
                obj.is_busy = true;
    
                %% Program Body
            
                % Go to initial position
                disp("Approaching initial position...")
                obj.setQ([pi/8;pi/8;pi/4;pi/4],true);
    
                % Plot desired position
                figure(obj.virtualRobot.fig);
                scatter3(x_desired(1),x_desired(2),x_desired(3), 30, 'filled', 'm');
    
                % Initialize the controller
                controller = NullspaceController(obj.virtualRobot);
    
                %% Control Loop
                
                breakTimerStarted = false;
                breakTimerValue = 0;

                while true
                    % Update virtual robot and plot
                    q = obj.realRobot.getQ;
                    obj.virtualRobot.setQ(q);
                    
                    % Compute the desired joint velocity
                    q_dot = controller.computeDesiredJointVelocity(x_desired, NaN, 0);
                            
                    % Update and draw the end-effector trajectory and the robot
                    obj.virtualRobot.updateTrajectory;
                    obj.virtualRobot.draw;
    
                    % Set q_dot to real Robot
                    obj.realRobot.setJointVelocities(q_dot);
                
                    % Print the distance to the goal
                    distance_to_goal = norm(x_desired-obj.virtualRobot.getEndeffectorPos);
                    fprintf('Distance to goal: %.0f mm \n', distance_to_goal);
                
                    if distance_to_goal < 2
                        if ~breakTimerStarted
                            % Start the break timer
                            breakTimerValue = tic;
                            breakTimerStarted = true;
                        elseif toc(breakTimerValue) >= 5
                            % 5 seconds have passed since the timer started
                            obj.realRobot.setJointVelocities([0; 0; 0; 0]);
                            obj.setQ([0;0;0;0],true);
                            obj.realRobot.torqueEnableDisable(0);
                            break;
                        end
                    else
                        breakTimerStarted = false;  % Reset the timer if the condition is no longer met
                    end
                end
    
                                
                %%
                if ~bypass_busy
                    obj.is_busy = false;
                end
            end
        end
    end


        %% END OF PROGRAMS


    

    methods (Access = private)

        function setupUpdateTimer(obj)
            % Setup and start the update timer
            obj.updateTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.1, ...
                                    'BusyMode', 'drop', ...
                                    'TimerFcn', @(~,~) obj.updateVirtualRobotAndPlot());
        end

        function updateVirtualRobotAndPlot(obj)
            % Timer callback function for updating and plotting
            if ~obj.is_busy

                % disp("Timer update")
                obj.is_busy = true;
                q = obj.realRobot.getQ;
                obj.virtualRobot.setQ(q);
                obj.virtualRobot.draw;
                obj.virtualRobot.frames(end).draw;
                obj.is_busy = false;
            end
        end
    end

    methods (Static)

        function initPath(obj)
            % Initialize the MATLAB path
            clc;
            close all;
            currentFile = mfilename('fullpath');
            [currentDir, ~, ~] = fileparts(currentFile);
            parentDir = fullfile(currentDir, '..');
            addpath(fullfile(parentDir, 'VirtualRobot'));
            addpath(fullfile(parentDir, 'Planner'));
            addpath(fullfile(parentDir, 'Controller'));
            addpath(fullfile(parentDir, 'RealRobot'));
        end           

    end

end
