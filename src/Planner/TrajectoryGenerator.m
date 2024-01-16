classdef TrajectoryGenerator < handle
    properties
        totalTime % [s]
        timeIncrement = 0.01; % [s]

        path % [mm] col: point

        x_d % [mm] col: point
        v_d % [mm/s] col: velocity
        t % [s] row
    end
    
    methods

        function obj = TrajectoryGenerator(path, totalTime)
            % Assuming waypoint_list is a 3xN matrix with each column representing a waypoint
            obj.path = path; % Combine X, Y, Z into a single matrix
            obj.totalTime = totalTime;

            obj.generateTrajectory;
        end
        

        function generateTrajectory(obj)
            disp("Generating Trajectory")
            
            % Create spline interpolations
            s = cumsum([0; sqrt(sum(diff(obj.path').^2, 2))]); % cumulative arclength
            totalTimesteps = ceil(obj.totalTime / obj.timeIncrement) + 1;
            s_d = linspace(0, s(end), totalTimesteps); % desired time steps
            x_d_spline = spline(s, obj.path(1,:), s_d);
            y_d_spline = spline(s, obj.path(2,:), s_d);
            z_d_spline = spline(s, obj.path(3,:), s_d);
            
            % Calculate desired velocities using finite differences
            v_x = diff(x_d_spline) / obj.timeIncrement;
            v_y = diff(y_d_spline) / obj.timeIncrement;
            v_z = diff(z_d_spline) / obj.timeIncrement;
            
            obj.t = obj.timeIncrement:obj.timeIncrement:totalTimesteps*obj.timeIncrement;
            obj.v_d = [v_x; v_y; v_z];
            obj.x_d = [x_d_spline; y_d_spline; z_d_spline];
            
            % Append zero vector to obj.v_d to finish with zero velocity.
            obj.v_d = [obj.v_d, [0;0;0]];
        end


        %% Plotting
        function draw(obj, fig)
            if isempty(fig)
                error("The TrajectoryGenerator draw method needs a figure specified" + ...
                    " (e.g. from a virtualRobot.fig object)")
            end

            % Draw the generated trajectory in the specified figure
            if ~isempty(obj.x_d)
                figure(fig);
                hold on;
                plot3(obj.x_d(1,:), obj.x_d(2,:), obj.x_d(3,:), 'm-', 'LineWidth', 1);
            else
                warning('TrajectoryGenerator: No trajectory data to plot.');
            end
        end

    end
end
