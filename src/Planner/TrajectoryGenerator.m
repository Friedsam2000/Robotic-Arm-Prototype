classdef TrajectoryGenerator < handle
    properties (Access = private)
        pathX % [mm] row vector
        pathY % [mm] row vector
        pathZ % [mm] row vector
        totalTime % [s]
        timeIncrement = 0.05; % [s]
        x_d % [mm] row: point
        v_d % [mm/s] row: velocity
        t % [s] row
    end
    
    methods

        function obj = TrajectoryGenerator(waypoint_list, totalTime)
            obj.pathX = waypoint_list(1,:);
            obj.pathY = waypoint_list(2,:);
            obj.pathZ = waypoint_list(3,:);
            obj.totalTime = totalTime;

            obj.generateTrajectory;
        end
        
        function [x_d, v_d, t] = getTrajectory(obj)
            x_d = obj.x_d;
            v_d = obj.v_d;
            t = obj.t;
        end

        function draw(obj, fig)

            if isempty(fig)
                error("The TrajectoryGenerator draw method needs a figure specified" + ...
                    " (e.g. from a virtualRobot.fig object)")
            end

            % Draw the generated trajectory in the specified figure
            if ~isempty(obj.x_d)
                figure(fig);  % Activate the specified figure
                hold on;  % Ensure that the plot does not overwrite existing plots
                plot3(obj.x_d(1,:), obj.x_d(2,:), obj.x_d(3,:), 'm-', 'LineWidth', 1);
            else
                warning('TrajectoryGenerator: No trajectory data to plot.');
            end
        end

    end

    methods (Access = private)

        function generateTrajectory(obj)

            disp("Generating Trajectory")
            
            % Create spline interpolations for X, Y, and Z
            s = cumsum([0; sqrt(diff(obj.pathX').^2 + diff(obj.pathY').^2 + diff(obj.pathZ').^2)]); % cumulative arclength
            totalTimesteps = ceil(obj.totalTime / obj.timeIncrement) + 1;
            
            s_d = linspace(0, s(end), totalTimesteps); % desired time steps based on the timeIncrement
            x_d_spline = spline(s, obj.pathX, s_d);
            y_d_spline = spline(s, obj.pathY, s_d);
            z_d_spline = spline(s, obj.pathZ, s_d);
            
            % Calculate desired velocities using finite differences
            v_x = diff(x_d_spline) / obj.timeIncrement;
            v_y = diff(y_d_spline) / obj.timeIncrement;
            v_z = diff(z_d_spline) / obj.timeIncrement;
            
            obj.t = obj.timeIncrement:obj.timeIncrement:totalTimesteps*obj.timeIncrement;
            obj.v_d = [v_x; v_y; v_z];
            obj.x_d = [x_d_spline; y_d_spline; z_d_spline];
            
            % Append zero vector to obj.v_d --> finish with zero velocity.
            obj.v_d = [obj.v_d, [0;0;0]];

        end

    end
end
