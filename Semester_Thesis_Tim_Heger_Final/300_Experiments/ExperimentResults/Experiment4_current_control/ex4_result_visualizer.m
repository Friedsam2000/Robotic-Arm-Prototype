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
path = "ExperimentResults/Experiment4_current_control/";
files = dir(path+"moment*.csv");

figure("WindowState", "Maximized");
hold on

colors = colororder();

visibility = "on";
%plot measurements
for file = files'
    m = readmatrix(path+file.name);
    plot(m(:,1), m(:,2), "DisplayName", "current control", "HandleVisibility",visibility, "Color",colors(1,:))
    visibility = "off";
end
%plot comparison
m2 = readmatrix(path+"ccw-3.csv");
plot(m2(:,1), m2(:,2), "DisplayName", "velocity control","Color",colors(2,:))

set(findall(gcf, 'Type', 'Line'), 'LineWidth', 1.5);
set(gca, "FontSize",30)
title("Current Control");
xlabel('Time [s]');
ylabel('Current Measurements [mA]');
legend("Location","Best");
grid on
