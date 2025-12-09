%% calculation

mass = 1.0561; % kg
g = 9.81; % m/s
radius = 0.1; % m

moment = mass*g*radius; % Nm

%from the graph in the datasheet (approximated):
function t = torque(c)
    t = c * (1.4 - 0.8)/(0.98 - 0.56);
end


%% helper function
function res = array_add(a,b)
    sa = size(a,1);
    sb = size(b,1);
    if sa < sb
        res = b + [a; zeros(sb-sa,1)];
    elseif sa > sb
        res = a + [b; zeros(sa-sb,1)];
    else
        res = a + b;
    end
end

%% visualisation
base_path = "ExperimentResults/dynamic_constant_1Nm/";
paths = [base_path+"dyn-cw*.csv", base_path+"dyn-ccw*.csv"];

colors = colororder();

figure("WindowState", "Maximized");
hold on
set(findall(gcf, 'Type', 'Line'), 'LineWidth', 1.5);
set(gca, "FontSize",30)
title("Applied Torque: "+moment+" Nm");
xlabel('Time [s]');
ylabel('Current Measurements [mA]');
ylim([20 900])
legend("Location","Best");
grid on

with_torque = [];
against_torque = [];

for j = 1:2
    path = paths(j);
    files = dir(path);
    
    torque_total = [];
    torque_amount = [];
    time_total = [];
    time_amount = [];
    
    visibility = "on";
    
    %plot measurements
    for file = files'
        file_path = string(file.folder) + "\" + string(file.name);
        table = readtable(file_path, "VariableNamingRule","preserve");
        names = string(table.Properties.VariableNames);
        m = readmatrix(file_path);
        i = 2;
        while (i <= size(names,2))

            %sort depending on filename
            torque_data = m(:,i);
            color = colors(1,:);
            name = "absolute clockwise measurements";
            if j == 1
                torque_data = -torque_data;
                color = colors(2,:);
                name = "absolute counterclockwise measurements";
            end

            %calculations
            for data_point = torque_data'
                if (data_point < 400) && (data_point > 200)
                    with_torque = [with_torque, data_point];
                elseif data_point > 600
                    against_torque = [against_torque, data_point];
                end
            end

            %plotting
            plot(m(:,1), torque_data, ".", "color", color, "DisplayName", name, "HandleVisibility",visibility)
            torque_total = array_add(torque_total,torque_data);
            torque_amount = array_add(torque_amount,ones(size(m,1),1));
            i = i+1;
            visibility = "off";
        end
        time_total = array_add(time_total,m(:,1));
        time_amount = array_add(time_amount,ones(size(m,1),1));
    end
    
    %plot average over measurements
    torque_average = torque_total./torque_amount;
    time_average = time_total./time_amount;
    %plot(time_average,torque_average, "LineWidth", 2, "DisplayName", "average measurement")
    
    %plot average over time (from 7s to 23s)
    % t = 7;
    % t_end = 23;
    % torque_value_total = 0;
    % torque_value_amount = 0;
    % for i = 1:length(torque_average)
    %     if time_average()
    
end

avg_with = mean(with_torque);
avg_against = mean(against_torque);

y1 = yline(avg_against);
y1.DisplayName = "average upward";
y1.Label = sprintf("%.0f",avg_against);
y1.Color = colors(5,:);
y1.LineWidth = 3;
y1.FontSize = 20;

y2 = yline(avg_with);
y2.DisplayName = "average downward";
y2.Label = sprintf("%.0f",avg_with);
y2.Color = colors(4,:);
y2.LineWidth = 3;
y2.FontSize = 20;

% dynamic_constant_with = moment/(avg_with*0.001)
% dynamic_constant_against = moment/(avg_against*0.001)
% factor = dynamic_constant_with/dynamic_constant_against