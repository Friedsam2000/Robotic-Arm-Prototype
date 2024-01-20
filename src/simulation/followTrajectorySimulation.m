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
trajectoryTime = 30; % s
trajectoryHeight = 400;

% Initialize the planner
planner = PathPlanner(virtualRobot);
planner.userInputPath(trajectoryHeight);
           


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
start_time = tic;
last_update_time = tic; % Track when the last update was done
desired_frame_time = 1/30; % Desired time per frame for 30 FPS

while true
    loop_start_time = tic;

    elapsed_time = toc(start_time);

    % Simulation
    q = virtualRobot.getJointAngles;

    [~, index] = min(abs(t - elapsed_time));
    if index >= length(t)
        fprintf("Trajectory time elapsed. \n");
        break;
    end

    % Get the desired position and velocity for the current timestep
    current_x_d = x_d(:, index);
    current_v_d = v_d(:, index);

    % Compute the desired joint velocity
    q_dot = controller.calcJointVelocities(current_x_d, current_v_d);

    % Ensure velocities do not exceed the configured maximum joint speed
    for ID = 1:4
        if abs(q_dot(ID)) > JOINT_VELOCITY_LIMITS(ID)
            q_dot(ID) = JOINT_VELOCITY_LIMITS(ID) * sign(q_dot(ID));
            % fprintf("Limited q_dot by sim : Joint %d \n", ID)
        end
    end

   % Check if enough time has passed since the last update
    if toc(last_update_time) >= desired_frame_time
        % Update and draw the robot
        virtualRobot.updateRobotPlot;
        drawnow

        % Reset the last update time
        last_update_time = loop_start_time;
    end

    % Calculate the time taken for this loop iteration
    dt = toc(loop_start_time);

    % Update the simulated robot's joint configuration
    virtualRobot.setJointAngles(q + q_dot * dt);

end
