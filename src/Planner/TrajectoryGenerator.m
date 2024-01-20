classdef TrajectoryGenerator < handle
    properties (Constant)
        timeIncrement = 0.01; % [s]
    end

    properties
        desiredPositionsArray = [];% [mm] col: point
        desiredVelocitiesArray = []; % [mm/s] col: velocity
        timeArray = []; % [s] row
    end
    
    methods

        function obj = TrajectoryGenerator()

        end
        

        function generateTrajectory(obj, path,duration)
            disp("Generating Trajectory")
            
            % Create spline interpolations
            s = cumsum([0; sqrt(sum(diff(path').^2, 2))]); % cumulative arclength
            durationsteps = ceil(duration / obj.timeIncrement) + 1;
            s_d = linspace(0, s(end), durationsteps); % Arclength array
            x_d_spline = spline(s, path(1,:), s_d); % Spline Interploation for positions x array
            y_d_spline = spline(s, path(2,:), s_d); % Spline Interploation for positions y array
            z_d_spline = spline(s, path(3,:), s_d); % Spline Interploation for positions z array
            
            % Calculate desired velocities using finite differences
            v_x = diff(x_d_spline) / obj.timeIncrement;
            v_y = diff(y_d_spline) / obj.timeIncrement;
            v_z = diff(z_d_spline) / obj.timeIncrement;
            
            obj.timeArray = obj.timeIncrement:obj.timeIncrement:durationsteps*obj.timeIncrement;
            obj.desiredVelocitiesArray = [v_x; v_y; v_z];
            obj.desiredPositionsArray = [x_d_spline; y_d_spline; z_d_spline];
            
            % Append zero vector to obj.v_d to finish with zero velocity.
            obj.desiredVelocitiesArray = [obj.desiredVelocitiesArray, [0;0;0]];
        end


        %% Plotting
        function draw(obj, fig)
            if isempty(fig)
                error("The TrajectoryGenerator draw method needs a figure specified" + ...
                    " (e.g. from a virtualRobot.fig object)")
            end

            % Draw the generated trajectory in the specified figure
            if ~isempty(obj.desiredPositionsArray)
                figure(fig);
                hold on;
                plot3(obj.desiredPositionsArray(1,:), obj.desiredPositionsArray(2,:), obj.desiredPositionsArray(3,:), 'm-', 'LineWidth', 1);
            else
                warning('TrajectoryGenerator: No trajectory data to plot.');
            end
        end

    end
end
