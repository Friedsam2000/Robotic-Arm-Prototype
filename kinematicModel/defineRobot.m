clear
clc
close

%% Setup Frames and Joints
orig_frame = CustomFrame([0; 0; 0], [], 'Origin');
joint1 = Joint([0; 0; 83.51], orig_frame, 'Joint 1', 'x');
joint2 = Joint([0;0;0], joint1, 'Joint 2', 'y');
joint3 = Joint([0;0;119.35], joint2, 'Joint 3', 'z');
joint4 = Joint([0;0;163.99], joint3, 'Joint 4', 'x');
endeffector_frame = CustomFrame([0;0;218.86], joint4, 'Endeffector');

%% Setup Links
link1 = CustomLink(orig_frame, joint1, 'r');  % Red
link2 = CustomLink(joint1, joint2, 'g');  % Green
link3 = CustomLink(joint2, joint3, 'b');  % Blue
link4 = CustomLink(joint3, joint4, 'y');  % Yellow
link5 = CustomLink(joint4, endeffector_frame, 'm');  % Magenta


%% Setup Robot
robot = Robot([joint1, joint2, joint3, joint4], [link1, link2, link3, link4, link5], [orig_frame, endeffector_frame]);


%% Main
% Set initial rotation
joint1.rotate(pi/4)
joint2.rotate(pi/4)
joint3.rotate(pi/4)
joint4.rotate(pi/4)

% Set a desired endeffector velocity
x_dot = [-50;0;0];

% Set a time increment for the simulation
dt = 0.1; % s

ref_positions_array = [];

for i = 1:1000

    % Get current joint angles % This would later be received from the real
    % motors. I should therefor add a setAngle method to the Joint to set
    % the angle from the real joints to the angle of the simulated joint.
    sAlpha = robot.joints(1).angle;
    sBeta = robot.joints(2).angle;
    sGamma = robot.joints(3).angle;
    sDelta = robot.joints(4).angle;

    % --> Set these angles to the modeled robot
    
    % Calculate the Jacobian in the current (modeled robot = real robot) configuration numerically
    J = robot.getJacobianNumeric;

    % Stop if the current configuration approaches a singularity
    if cond(pinv(J)) > 15
        disp('Warning: Close to singularity!');
        break
    end

    % Calculate respective joint velocity
    q_dot = pinv(J) * x_dot;

    % Set the joint velocity to the real robot here
    % Check for further constraints such as joint limits and elevation
    % limits for the bevel gear..

    %Apply joint rotation increment % No need for this later since the
    %simulated joints will be set by the real robot as mentioned above
    robot.joints(1).rotate(q_dot(1)*dt);
    robot.joints(2).rotate(q_dot(2)*dt);
    robot.joints(3).rotate(q_dot(3)*dt);
    robot.joints(4).rotate(q_dot(4)*dt);

    % Visualize the robot every x frame % Visualize the modeld robot that
    % should be a mirror image of the real robot
    clearFig = 0;
    draw_frames = 0;
    robot.display(clearFig, draw_frames);
    drawnow

    % Get endeffector info
    display_info = 0;
    [ref_position, ref_rotation, ref_frame] = endeffector_frame.getInfo(display_info);

    % Append the endeffector postion to an array and plot the trajectory
    ref_positions_array = [ref_positions_array ref_position];
    plot3(ref_positions_array(1,:),ref_positions_array(2,:),ref_positions_array(3,:),'k');

    % fprintf('Time for iteration %d: %f seconds\n', i, loopTime);
    

end
