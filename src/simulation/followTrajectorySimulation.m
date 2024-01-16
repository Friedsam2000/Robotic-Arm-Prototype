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
plannerDir = fullfile(parentDir, 'Planner'); 
controllerDir = fullfile(parentDir, 'Controller'); 

% Add these paths to the MATLAB path
addpath(virtualRobotDir);
addpath(plannerDir);
addpath(controllerDir);

%% Setup virtual robot, controller, planner, and trajectory generator

virtualRobot = VirtualRobot;

% Set the virtualRobot to a non-singularity position
virtualRobot.setJointAngles([pi/8;pi/8;pi/4;pi/4])

% Initialize the controller
controller = NullspaceController(virtualRobot);

% Conifgure maximum absolut joint velocities % RAD/s
JOINT_VELOCITY_LIMITS = [0.6;0.6;2;2];

%% Create a trajectory
trajectoryTime = 10; % s
trajectoryHeight = 400;

% Initialize the planner
planner = PathPlanner(virtualRobot, trajectoryHeight);

% Initialize the trajectory generator
trajectoryGenerator = TrajectoryGenerator();
trajectoryGenerator.generateTrajectory(planner.path,trajectoryTime)
x_d = trajectoryGenerator.desiredPositionsArray;
v_d = trajectoryGenerator.desiredVelocitiesArray;
t = trajectoryGenerator.timeArray;

% Plot the desired trajectory
virtualRobot.initRobotPlot
trajectoryGenerator.draw(virtualRobot.fig)


%% Control Loop

% Variables for time tracking
dt = 0.1;

index = 0;
while true
    index = index +10;
    % Simulation
    q = virtualRobot.getJointAngles;

    % Break the loop if the end of the trajectory is reached
    if index >= length(t)
        break;
    end

    % Get the desired position and velocity for the current timestep
    current_x_d = x_d(:, index);
    current_v_d = v_d(:, index);

    % Compute the desired joint velocity
    q_dot = controller.calcJointVelocity(current_x_d, current_v_d);

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

    drawnow limitrate
    pause(0.1)

end
