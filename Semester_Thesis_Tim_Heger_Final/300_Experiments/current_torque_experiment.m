%% experiment to determine current reading based on applied torque
% apply constant torque -> run experiment -> analyze current reading
local_init;
state = 6; %set type of experiment: 
            % 1:    vel_cw
            % 2:    vel_ccw
            % 3:    pos_cw
            % 4:    pos_ccw
            % 5:    dyn_cw
            % 6:    dyn_ccw
            % else: test mode

calculate_torque = false;

%% test mode settings
test_vel = -47;
test_pos = 7;
test_mode = 1; %1:velocity control, 4:extend position control
save_test_data = true;

%% timestep settings:
iterations = 100;
delta_t = 10;
target_vel = -3;
target_pos = 1;

save_data = true;
show_figure = true;

%% initialisation
local_init
filename = char(datetime("now","Format",'yy-MM-dd''-T''HHmmss'))+".csv";

vel = test_vel;
pos = test_pos;
mode = test_mode;

switch state
    case 1
        filename = "vel-cw-"+filename;
        vel = target_vel;
        mode = 1;
    case 2
        filename = "vel-ccw-"+filename;
        vel = -target_vel;
        mode = 1;
    case 3
        filename = "pos-cw-"+filename;
        pos = -target_pos;
        mode = 4;
    case 4
        filename = "pos-ccw-"+filename;
        pos = target_pos;
        mode = 4;
    case 5
        filename = "dyn-cw-"+filename;
        vel = target_vel;
        mode = 1;
    case 6
        filename = "dyn-ccw-"+filename;
        vel = -target_vel;
        mode = 1;
    otherwise
        filename = "cw-"+filename;
        save_data = save_test_data;
end


%% main experiment
servo = ServoChain.getInstance(lib_path, port);
servo_ids = servo.availableIDs;
servo_pos = servo.getServoAngles();
n = length(servo_ids);
velocities = vel.*ones(n,1);
positions = pos.*ones(n,1);
names = strings(1,n+1);
i = 1;
names(i) = "time";
for id = servo_ids
    if (servo.getServoOperationMode(id) ~= mode)
        if (servo.getServoTorque(id) ~= 0)
            servo.setServoTorque(id,0)
        end
        servo.setServoOperationMode(id,mode)%activate desired control mode
    end
    if (servo.getServoTorque(id) ~= 1)
        servo.setServoTorque(id,1)
    end
    i=i+1;
    if calculate_torque
        names(i) = "torque servo"+id;
    else
        names(i) = "current servo"+id;
    end
end

tic
if (state == 1 || state == 2 || state == 5 || state == 6)
    servo_measurements = zeros(3*iterations,n);
    servo_time = zeros(3*iterations,1);
    %forward:
    servo.setServoVelocities(velocities)
    while i <= iterations
        if calculate_torque
            servo_measurements(i,:) = servo.getServoMeasuredTorques;
        else
            servo_measurements(i,:) = servo.getServoCurrents;
        end
        servo_time(i) = toc;
        i = 1+i;
        pause(0.1)
    end
    if (state == 1 || state == 2)
        %still
        servo.setServoVelocities(zeros(n))
    else
        %backward
        servo.setServoVelocities(-velocities)
    end
    i=0;
    while i <= iterations/2
        % if (i==10)
        %     servo.setServoVelocities(zeros(n))
        % end
        if calculate_torque
            servo_measurements(i+iterations,:) = servo.getServoMeasuredTorques;
        else
            servo_measurements(i+iterations,:) = servo.getServoCurrents;
        end
        servo_time(i+iterations) = toc;
        i = 1+i;
        pause(0.1)
    end
    if (state == 1 || state == 2)
        %backward
        servo.setServoVelocities(-velocities)
    else
        %forward
        servo.setServoVelocities(velocities)
    end
    i=0;
    while i <= iterations/2
        % if (i == 10)
        %     servo.setServoVelocities(-velocities)
        % end
        if calculate_torque
            servo_measurements(i+1.5*iterations,:) = servo.getServoMeasuredTorques;
        else
            servo_measurements(i+1.5*iterations,:) = servo.getServoCurrents;
        end
        servo_time(i+1.5*iterations) = toc;
        i = 1+i;
        pause(0.1)
    end
    if (state == 1 || state == 2)
        %forward
        servo.setServoVelocities(velocities)
    else
        %backward
        servo.setServoVelocities(-velocities)
    end
    i=0;
    while i <= iterations/2
        % if (i==10)
        %     servo.setServoVelocities(zeros(n))
        % end
        if calculate_torque
            servo_measurements(i+2*iterations,:) = servo.getServoMeasuredTorques;
        else
            servo_measurements(i+2*iterations,:) = servo.getServoCurrents;
        end
        servo_time(i+2*iterations) = toc;
        i = 1+i;
        pause(0.1)
    end
    %backward again
    servo.setServoVelocities(-velocities)
    i=0;
    while i <= iterations/2
        % if (i == 10)
        %     servo.setServoVelocities(-velocities)
        % end
        if calculate_torque
            servo_measurements(i+2.5*iterations,:) = servo.getServoMeasuredTorques;
        else
            servo_measurements(i+2.5*iterations,:) = servo.getServoCurrents;
        end
        servo_time(i+2.5*iterations) = toc;
        i = 1+i;
        pause(0.1)
    end
elseif (state == 3 || state == 4)
    servo_measurements = zeros(iterations,n);
    servo_time = zeros(iterations,1);
    servo.setServoPositions(servo_pos + positions)
    while i <= iterations
        if (i == iterations - 15)
            servo.setServoPositions(servo_pos)
        end
        if calculate_torque
            servo_measurements(i,:) = servo.getServoMeasuredTorques;
        else
            servo_measurements(i,:) = servo.getServoCurrents;
        end
        servo_time(i) = toc;
        i = 1+i;
        pause(0.1)
    end
else
    servo_measurements = zeros(iterations,n);
    servo_time = zeros(iterations,1);
    if (mode == 1)
        servo.setServoVelocities(velocities)
    elseif (mode == 3)
        servo.setServoPositions(positions)
    end
    while i <= iterations
        if calculate_torque
            servo_measurements(i,:) = servo.getServoMeasuredTorques;
        else
            servo_measurements(i,:) = servo.getServoCurrents;
        end
        servo_time(i) = toc;
        i = 1+i;
        pause(0.1)
    end
end
if (mode == 1)
    servo.setServoVelocities(zeros(n))
    servo.setServoTorque(id,0)
    servo.setServoOperationMode(id,3)
    servo.setServoTorque(id,1)
end
if (servo.getServoOperationMode(id) == 3)
    servo.setServoPositions(servo_pos)
end
servo.delete


%% create figure
if (show_figure)
    figure("WindowState", "Maximized");
        
    hold on
    grid on
    for i = 1:n
        plot(servo_time, servo_measurements(:,i), "-", "LineWidth", 2, "DisplayName", "servo "+servo_ids(i))
        mean1 = mean(servo_measurements(:,i));
        %yline(mean1,"-", mean1, "DisplayName", "average servo "+servo_ids(i))
    end
    hold off
    
    title('Measurements at Constant Applied Torque', "FontSize", 20);
    xlabel('Time (s)', "FontSize", 18);
    ylabel('Measurement', "FontSize", 18);
    legend("FontSize", 16);
end

%% save data
csv_matrix = [names; servo_time, servo_measurements];
if (save_data)
    writematrix(csv_matrix,"ExperimentResults\"+filename)
end