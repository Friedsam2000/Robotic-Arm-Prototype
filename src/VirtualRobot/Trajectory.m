classdef Trajectory < handle

    properties (Constant)
        Length = 30; % Constant length of the trajectory
    end

    properties

        Points;       % Stores the trajectory points

        Index;    % Current index in the trajectory buffer

        % Plotting
        PlotHandle;   % Handle for the trajectory plot
    end

    methods
        %% Constructor
        function obj = Trajectory()
            obj.Points = zeros(3, obj.Length); % Initialize with zeros
        end

        %% Plotting
        function initTrajectoryPlot(obj, currentPosition)
            obj.Index = 1;
            obj.Points(:, :) = repmat(currentPosition, 1, obj.Length);
            obj.PlotHandle = plot3(obj.Points(1, :), obj.Points(2, :), obj.Points(3, :), 'k-', 'LineWidth', 1);
        end

        % Method to update trajectory
        function updateTrajectoryPlot(obj, currentPosition)
            obj.Points(:, obj.Index) = currentPosition;
            obj.Index = mod(obj.Index, obj.Length) + 1;
            rolledPoints = circshift(obj.Points, [0, -obj.Index + 1]);
            set(obj.PlotHandle, 'XData', rolledPoints(1, :), 'YData', rolledPoints(2, :), 'ZData', rolledPoints(3, :));
        end
    end
end
