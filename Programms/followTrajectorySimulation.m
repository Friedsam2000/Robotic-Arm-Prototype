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

%% Setup simulated robot, controller, planner, and trajectory generator

virtualRobot = VirtualRobot;

% Set the virtualRobot to a non-singularity position
virtualRobot.setQ([0.3; 0.3; 0.5; 0.5])

% Initialize the controller
controller = NullspaceController(virtualRobot);
 
%% Create a trajectroy
trajectory_time = 5; % s
traj_z = 400;

% Initialize the planner
planner = PathPlanner2D(virtualRobot, traj_z);

% Prompt the user to draw a path
planner.userInputPath;

% Initialize the trajectory generator
trajectoryGenerator = TrajectoryGenerator(planner.getWaypointList, trajectory_time);
[x_d, v_d, t] = trajectoryGenerator.getTrajectory;
total_timesteps = length(t);

% Plot the desired trajectory
virtualRobot.draw
plot3(x_d(1,:),x_d(2,:),x_d(3,:),'m');
figure(virtualRobot.fig);

% Plot the workspace
virtualRobot.workspace.draw;

%% Control Loop

% Init array for storing tcp positions
tcp_positions = zeros(3, total_timesteps);

% Variables for time tracking
loopBeginTime = tic;  % Start the timer
previousTime = 0;  % Initialize previous time
currentStep = 1;  % Initialize the step counter

while true
    % Simulation
    q = virtualRobot.getQ;
    % Display the virtualRobot
    tcp_positions(:, currentStep) = virtualRobot.getEndeffectorPos;
    plot3(tcp_positions(1, 1:currentStep), tcp_positions(2, 1:currentStep), tcp_positions(3, 1:currentStep), 'k');
    virtualRobot.draw;
    drawnow limitrate

    % Calculate elapsed time and the time increment (dt)
    elapsedRealTime = toc(loopBeginTime);
    dt = elapsedRealTime - previousTime;
    previousTime = elapsedRealTime;  % Update the previous time for the next loop iteration

    % Find the index in the trajectory that corresponds to the elapsed time
    [~, index] = min(abs(t - elapsedRealTime));

    % Break the loop if the end of the trajectory is reached
    if index >= total_timesteps
        break;
    end

    % Get the desired position and velocity for the current timestep
    current_x_d = x_d(:, index);
    current_v_d = v_d(:, index);

    % Compute the desired joint velocity
    q_dot = controller.computeDesiredJointVelocity(virtualRobot, current_x_d, NaN, current_v_d);

    % Update the robot's joint configuration
    virtualRobot.setQ(q + q_dot * dt);

    % Check if the current step needs to be incremented
    if elapsedRealTime >= t(currentStep)
        currentStep = currentStep + 1;
    end
end
