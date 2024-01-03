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

% Set the virtualRobot to a non-singularity position
virtualRobot.setQ([0.3; 0.3; 0.5; 0.5])

% Initialize the controller
controller = NullspaceController(virtualRobot);
 
%% Create a trajectory
trajectory_time = 15; % s
trajectory_height = 400;

% Initialize the planner
planner = PathPlanner2D(virtualRobot, trajectory_height);

% Prompt the user to draw a path
planner.userInputPath;

% Initialize the trajectory generator
trajectoryGenerator = TrajectoryGenerator(planner.getWaypointList, trajectory_time);
[x_d, v_d, t] = trajectoryGenerator.getTrajectory;

% Plot the desired trajectory
virtualRobot.draw
trajectoryGenerator.draw(virtualRobot.fig)

% Plot the workspace
virtualRobot.workspace.draw;

%% Control Loop

% Variables for time tracking
loopBeginTime = tic;  % Start the timer
previousTime = 0;  % Initialize previous time

while true
    % Simulation
    q = virtualRobot.getQ;
    
    % Calculate elapsed time and the time increment (dt)
    elapsedRealTime = toc(loopBeginTime);
    dt = elapsedRealTime - previousTime;
    previousTime = elapsedRealTime;  % Update the previous time for the next loop iteration

    % Find the index in the trajectory that corresponds to the elapsed time
    [~, index] = min(abs(t - elapsedRealTime));

    % Break the loop if the end of the trajectory is reached
    if index >= length(t)
        break;
    end

    % Get the desired position and velocity for the current timestep
    current_x_d = x_d(:, index);
    current_v_d = v_d(:, index);

    % Compute the desired joint velocity
    q_dot = controller.computeDesiredJointVelocity(current_x_d, NaN, current_v_d);

    % Update the simulated robot's joint configuration
    virtualRobot.setQ(q + q_dot * dt);

    % Update and draw the end-effector trajectory and the robot
    virtualRobot.updateTrajectory;
    virtualRobot.draw;

end
