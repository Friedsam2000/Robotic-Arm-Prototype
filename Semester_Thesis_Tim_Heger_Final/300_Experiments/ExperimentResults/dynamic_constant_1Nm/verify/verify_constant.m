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
base_path = "ExperimentResults/dynamic_constant/verify/";
paths = [base_path+"dyn-cw*.csv", base_path+"dyn-ccw*.csv"];

figure("WindowState", "Maximized");
hold on
title("Applied Torque: "+moment+" Nm", "FontSize", 20);
xlabel('Time (s)', "FontSize", 18);
ylabel('Measurements', "FontSize", 18);
%legend("FontSize", 16,"Location","Best");
grid on

neg_torque = [];
pos_torque = [];

for j = 1:2
    path = paths(j);
    files = dir(path);
    
    torque_total = [];
    torque_amount = [];
    time_total = [];
    time_amount = [];
    
    
    %plot measurements
    for file = files'
        file_path = string(file.folder) + "\" + string(file.name);
        table = readtable(file_path, "VariableNamingRule","preserve");
        names = string(table.Properties.VariableNames);
        m = readmatrix(file_path);
        i = 2;
        while (i <= size(names,2))

            torque_data = m(:,i);

            %calculations
            for data_point = torque_data'
                if data_point < -0.5
                    neg_torque = [neg_torque, data_point];
                elseif data_point > 0.5
                    pos_torque = [pos_torque, data_point];
                end
            end

            %plotting
            plot(m(:,1), torque_data, "-", "DisplayName", names(i)+" "+file.name)
            torque_total = array_add(torque_total,torque_data);
            torque_amount = array_add(torque_amount,ones(size(m,1),1));
            i = i+1;
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

avg_neg = mean(neg_torque);
avg_pos = mean(pos_torque);
yline(avg_pos, "-", avg_pos)
yline(avg_neg, "-", avg_neg)
