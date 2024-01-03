%% Setup workspace and matlab PATH
clear
clc
close all

% Get the directory of the currently executing script
currentFile = mfilename('fullpath');
[currentDir, ~, ~] = fileparts(currentFile);

% Construct the paths to the folders one level up
parentDir = fullfile(currentDir, '..');  % This goes one level up
virtualRobotDir = fullfile(parentDir, 'VirtualRobot');
plannerDir = fullfile(parentDir, 'Planner'); 
controllerDir = fullfile(parentDir, 'Controller'); 

% Add these paths to the MATLAB path
addpath(virtualRobotDir);
addpath(plannerDir);
addpath(controllerDir);

%% Setup virtual robot, controller, planner, and trajectory generator

virtualRobot = VirtualRobot;
realRobot = RealRobot;

% Initial position setup for Real robot
realRobot.setZeroPositionToCurrentPosition;
realRobot.torqueEnableDisable(1);
realRobot.setJointVelocities([0.02,0.02,0.1,0.1]);
pause(1)
realRobot.setJointVelocities([0,0,0,0]);

% Set simulated Robot to same config as real robot
virtualRobot.setQ(realRobot.getQ);

% Initialize the controller
controller = NullspaceController(virtualRobot);
 
% Desired position
x_desired = [100;100;500];
virtualRobot.draw
scatter3(x_desired(1),x_desired(2),x_desired(3), 'm', 'filled');

% Plot the workspace
virtualRobot.workspace.draw;

%% Control Loop

% Variables for time tracking
loopBeginTime = tic;  % Start the timer
previousTime = 0;  % Initialize previous time

breakTimerStarted = false;
breakTimerValue = 0;

while true
    % Update virtual robot and plot
    q = realRobot.getQ;
    virtualRobot.setQ(q);
    
    % Compute the desired joint velocity
    q_dot = controller.computeDesiredJointVelocity(x_desired, NaN, 0);
            
    % Update and draw the end-effector trajectory and the robot
    virtualRobot.updateTrajectory;
    virtualRobot.draw;
    
    % Set q_dot to real Robot
    realRobot.setJointVelocities(q_dot);
    
    % Print the distance to the goal
    distance_to_goal = norm(x_desired-virtualRobot.getEndeffectorPos);
    fprintf('Distance to goal: %.0f mm \n', distance_to_goal);
    

    if distance_to_goal < 2
        if ~breakTimerStarted
            % Start the break timer
            breakTimerValue = tic;
            breakTimerStarted = true;
        elseif toc(breakTimerValue) >= 5
            % 5 seconds have passed since the timer started
            realRobot.setJointVelocities([0; 0; 0; 0]);
            break;
        end
    else
        breakTimerStarted = false;  % Reset the timer if the condition is no longer met
    end
end


