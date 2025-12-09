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

figure("WindowState", "Maximized");
hold on

paths = ["ExperimentResults/planetary_gearbox/with_silicon/","ExperimentResults/planetary_gearbox/without_silicon/"];

for path = paths
    category_array = strsplit(path, "/");
    category = category_array(length(category_array)-1);
    directions = ["ccw"];

    for direction = directions
        files = dir(path+direction+"*.csv");
        
        torque_total = [];
        torque_amount = [];
        time_total = [];
        time_amount = [];
        
        %plot measurements
        for file = files'
            table = readtable(path+file.name, "VariableNamingRule","preserve");
            names = string(table.Properties.VariableNames);
            m = readmatrix(path+file.name);
            i = 2;
            while (i <= size(names,2))
                %plot(m(:,1), m(:,i),".", "color", "#808080","HandleVisibility","off")%, "DisplayName", names(i)+" "+file.name)
                torque_total = array_add(torque_total,m(:,i));
                torque_amount = array_add(torque_amount,ones(size(m,1),1));
                i = i+1;
            end
            time_total = array_add(time_total,m(:,1));
            time_amount = array_add(time_amount,ones(size(m,1),1));
        end
        
        %plot average over measurements
        torque_average = torque_total./torque_amount;
        time_average = time_total./time_amount;
        visibility = "on";
        if direction == "cw"
            visibility = "off";
        end
        color = "blue";
        if category == "with_silicon"
            color = "red";
        end
        plot(time_average,torque_average, "LineWidth", 3, "DisplayName", "average "+strrep(category,"_"," ")+"e spray","HandleVisibility",  visibility, "Color", color)
    end
end

set(gca, "FontSize",30)
title("Running the Planetary Gearbox");
xlabel('Time [s]');
xlim([1 12])
ylim([0 0.1])
ylabel('Measured Current [mA]');
legend("Location","Best");
grid on
