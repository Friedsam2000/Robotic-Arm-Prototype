%% calculation

mass = 1.0561; % kg
g = 9.81; % m/s
radius = 0.1; % m

moment = mass*g*radius; % Nm

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
path = "ExperimentResults/Experiment2_lifting_weight/";
files = dir(path+"*.csv");

torque_total = [];
torque_amount = [];
time_total = [];
time_amount = [];
name = "constant weight";

figure("WindowState", "Maximized");
hold on

%plot measurements
for file = files'
    table = readtable(path+file.name, "VariableNamingRule","preserve");
    names = string(table.Properties.VariableNames);
    m = readmatrix(path+file.name);
    i = 2;
    while (i <= size(names,2))
        plot(m(:,1), m(:,i), "DisplayName", name)
        torque_total = array_add(torque_total,m(:,i));
        torque_amount = array_add(torque_amount,ones(size(m,1),1));
        i = i+1;
    end
    time_total = array_add(time_total,m(:,1));
    time_amount = array_add(time_amount,ones(size(m,1),1));
    name = "weight reduction";
end

set(findall(gcf, 'Type', 'Line'), 'LineWidth', 1.5);
set(gca, "FontSize",30)
title("Temporary Weight Reduction");
xlabel('Time [s]');
ylabel('Current Measurements [mA]');
legend("Location","Best");
grid on