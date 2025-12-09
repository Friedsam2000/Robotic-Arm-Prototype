%% calculation

base_path = "ExperimentResults/dynamic_constant_xNm/";
paths = dir(base_path);
paths.name;

timestamps = [1,11,13,17,19.5,23,26,32.5];
profile = [0,1,0,-1,0,1,0,-1,0];
colors = colororder();

with_torque = [];
against_torque = [];

for path = paths'
    if (path.isdir && ~ismember(path.name, [".",".."]))
        written_torque = str2double(strrep(path.name,"_","."));

        files = dir(path.folder+"\"+path.name+"\*.csv");
        for file = files'
            file_path = string(file.folder) + "\" + string(file.name);

            table = readtable(file_path, "VariableNamingRule","preserve");
            names = string(table.Properties.VariableNames);
            m = readmatrix(file_path);
            time_vector = m(:,1);
            i = 2;
            while (i <= size(names,2))
    
                %sort depending on filename
                measurements = [];
                torque = 0;
                if contains(file.name,"dyn-ccw")
                    measurements = m(:,i);
                    torque = written_torque;
                elseif contains(file.name,"dyn-cw")
                    %measurements = -m(:,i);
                    measurements = m(:,i);
                    torque = -written_torque;
                end
                p = 1;
                for j = 1:length(measurements)
                    switch profile(p)
                        case -1
                            with_torque = [with_torque,[measurements(j);torque]];

                        case 1
                            against_torque = [against_torque,[measurements(j);torque]];
                    end
                    if p<9
                        if time_vector(j) >= timestamps(p)
                            p = p+1;
                        end
                    end
                end
                i = i+1;
            end
        end
    end
end

%% visualisation

figure("WindowState", "Maximized");
hold on
grid on

fitted_with = fit(with_torque(2,:)',with_torque(1,:)',"poly1");
fitted_against = fit(against_torque(2,:)',against_torque(1,:)',"poly1");

plot(against_torque(2,:),against_torque(1,:),".","color",colors(1,:),"HandleVisibility","off")

handle_with = plot(fitted_with);
handle_against = plot(fitted_against);

plot(with_torque(2,:),with_torque(1,:),".","color",colors(3,:),"DisplayName","measurements with torque direction")
plot(against_torque(2,:),against_torque(1,:),".","color",colors(4,:),"DisplayName","measurements against torque direction")


set(handle_with,"DisplayName","fitted line with torque direction")
set(handle_against,"DisplayName","fitted line against torque direction","Color",colors(1,:))

set(findall(gcf, 'Type', 'Line'), 'LineWidth', 2);
set(gca, "FontSize",30)
title("Current-Torque Relation");
xlabel('Applied Torque [Nm]');
ylabel('Measured Current [mA]');
legend("Location","Best");
