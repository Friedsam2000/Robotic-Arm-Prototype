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
path = "ExperimentResults/Experiment3_radial_load/";
files = dir(path+"*.csv");

torque_total = [];
torque_amount = [];
time_total = [];
time_amount = [];

figure("WindowState", "Maximized");
hold on

visibility = "on";
color = "red";
%plot measurements
for file = files'
    table = readtable(path+file.name, "VariableNamingRule","preserve");
    names = string(table.Properties.VariableNames);
    m = readmatrix(path+file.name);
    i = 2;
    name = "no radial load";
    if contains(file.name, "rl")
        name = "radial load";
        if color == "red"
            visibility = "on";
        end
        color = "blue";
    end
    while (i <= size(names,2))
        plot(m(:,1), m(:,i), "DisplayName", name, "Color", color, "HandleVisibility", visibility)
        torque_total = array_add(torque_total,m(:,i));
        torque_amount = array_add(torque_amount,ones(size(m,1),1));
        i = i+1;
        visibility = "off";
    end
    time_total = array_add(time_total,m(:,1));
    time_amount = array_add(time_amount,ones(size(m,1),1));
end

set(findall(gcf, 'Type', 'Line'), 'LineWidth', 1.5);
set(gca, "FontSize",30)
title("Radial Load Influence");
xlabel('Time [s]');
ylabel('Current Measurements [mA]');
legend("Location","Best");
grid on
