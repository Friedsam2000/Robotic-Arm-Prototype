%% calculation

mass = 1.0561; % kg
g = 9.81; % m/s
radius = 0.1; % m

moment = mass*g*radius; % Nm

%from the graph in the datasheet (approximated):
function t = torque(c)
    t = c;% * (1.4 - 0.8)/(0.98 - 0.56) *0.001;
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
base_path = "ExperimentResults/compare_motors/*/";
paths = [base_path+"vel-cw*.csv", base_path+"vel-ccw*.csv"];%, base_path+"pos-cw*.csv", base_path+"pos-ccw*.csv"];

figure("WindowState", "Maximized");
titles = ["Counterclockwise External Torque", "Clockwise External Torque"];
j=2;
for path = paths
    files = dir(path);
    
    torque_total = [];
    torque_amount = [];
    time_total = [];
    time_amount = [];
    used_names = [""];
    colors = colororder();
    cur_col = 0;

    subplot(2,1,j)
    hold on
    set(gca, "FontSize",30)

    %plot measurements
    for file = files'
        file_path = string(file.folder) + "\" + string(file.name);
        table = readtable(file_path, "VariableNamingRule","preserve");
        names = string(table.Properties.VariableNames);
        m = readmatrix(file_path);
        i = 2;
        while (i <= size(names,2))
            torque_data = torque(m(:,i));
            if ismember(names(i),used_names)
                plot(m(:,1), torque_data,"color", colors(cur_col,:), "HandleVisibility","off")
            else
                cur_col =cur_col+1;
                plot(m(:,1), torque_data,"color", colors(cur_col,:), "DisplayName", names(i).replace("current ",""))
                used_names = [used_names,names(i)];
            end
            torque_total = array_add(torque_total,torque_data);
            torque_amount = array_add(torque_amount,ones(size(m,1),1));
            i = i+1;
        end
        time_total = array_add(time_total,m(:,1));
        time_amount = array_add(time_amount,ones(size(m,1),1));
    end
    
    set(findall(gcf, 'Type', 'Line'), 'LineWidth', 1.5);
    title(titles(j))
    xlabel('Time [s]');
    ylabel('Current [mA]');
    legend("Location","Best");
    grid on
    j = j-1;
end
