%% Setup workspace and matlab PATH
clear
clc
close all

% Get the directory of the currently executing script
currentFile = mfilename('fullpath');
[currentDir, ~, ~] = fileparts(currentFile);

% Construct the paths to the folders one level up
parentDir = fullfile(currentDir, '..');  % This goes one level up to src
virtualRobotDir = fullfile(parentDir, 'VirtualRobot');
controllerDir = fullfile(parentDir, 'Controller'); 

% Add these paths to the MATLAB path
addpath(virtualRobotDir);
addpath(controllerDir);

%% Setup virtual robot, controller, planner, and trajectory generator

virtualRobot = VirtualRobot;

% Set the virtualRobot to a non-singularity position
virtualRobot.setJointAngles([0.5;0.5;0.5;0.5])


% Initialize the controller
controller = NullspaceController(virtualRobot);

% Conifgure maximum absolut joint velocities % RAD/s
JOINT_VELOCITY_LIMITS = [0.6;0.6;2;2];
x_desired = [100; 100; 400];
virtualRobot.initRobotPlot;
scatter3(x_desired(1), x_desired(2), x_desired(3), 'm', 'filled');


%% Control Loop

% Variables for time tracking
loopBeginTime = tic;  % Start the timer
previousTime = 0;  % Initialize previous time

dt = 0.1;

while true
    % Simulation
    q = virtualRobot.getJointAngles;
    
    % Compute the desired joint velocity
    q_dot = controller.calcJointVelocity(x_desired, 0);

    % Ensure velocities do not exceed the configued maximum joint speed
    for ID = 1:4
        if abs(q_dot(ID)) > JOINT_VELOCITY_LIMITS(ID)
            q_dot(ID) = JOINT_VELOCITY_LIMITS(ID) * sign(q_dot(ID));
            fprintf("Limited q_dot by sim : Joint %d \n", ID)
        end
    end

    % Update the simulated robot's joint configuration
    virtualRobot.setJointAngles(q + q_dot * dt);

    % Update and draw the end-effector trajectory and the robot
    virtualRobot.updateRobotPlot;

    pause(0.1)
    drawnow limitrate

end
