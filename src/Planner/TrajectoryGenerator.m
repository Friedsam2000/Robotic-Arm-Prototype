classdef TrajectoryGenerator < handle
    properties (Constant)
        timeIncrement = 0.01; % [s]
    end

    properties
        desiredPosition % [mm] col: point
        desiredVelocity % [mm/s] col: velocity
        t % [s] row
    end
    
    methods

        function obj = TrajectoryGenerator()

        end
        

        function generateTrajectory(obj, path,trajectoryTime)
            disp("Generating Trajectory")
            
            % Create spline interpolations
            s = cumsum([0; sqrt(sum(diff(path').^2, 2))]); % cumulative arclength
            trajectoryTimesteps = ceil(trajectoryTime / obj.timeIncrement) + 1;
            s_d = linspace(0, s(end), trajectoryTimesteps); % desired time steps
            desiredPosition_spline = spline(s, path(1,:), s_d);
            y_d_spline = spline(s, path(2,:), s_d);
            z_d_spline = spline(s, path(3,:), s_d);
            
            % Calculate desired velocities using finite differences
            v_x = diff(desiredPosition_spline) / obj.timeIncrement;
            v_y = diff(y_d_spline) / obj.timeIncrement;
            v_z = diff(z_d_spline) / obj.timeIncrement;
            
            obj.t = obj.timeIncrement:obj.timeIncrement:trajectoryTimesteps*obj.timeIncrement;
            obj.desiredVelocity = [v_x; v_y; v_z];
            obj.desiredPosition = [desiredPosition_spline; y_d_spline; z_d_spline];
            
            % Append zero vector to obj.desiredVelocity to finish with zero velocity.
            obj.desiredVelocity = [obj.desiredVelocity, [0;0;0]];
        end


        %% Plotting
        function draw(obj, fig)
            if isempty(fig)
                error("The TrajectoryGenerator draw method needs a figure specified" + ...
                    " (e.g. from a virtualRobot.fig object)")
            end

            % Draw the generated trajectory in the specified figure
            if ~isempty(obj.desiredPosition)
                figure(fig);
                hold on;
                plot3(obj.desiredPosition(1,:), obj.desiredPosition(2,:), obj.desiredPosition(3,:), 'm-', 'LineWidth', 1);
            else
                warning('TrajectoryGenerator: No trajectory data to plot.');
            end
        end

    end
end
