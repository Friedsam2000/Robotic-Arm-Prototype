classdef Trajectory < handle

    properties (Constant)
        Length = 30; % Constant length of the trajectory
    end

    properties

        storedPositions = [];       % Stores the trajectory

        Index = [];    % Current index in the trajectory buffer

        % Plotting
        PlotHandle = [];   % Handle for the trajectory plot
    end

    methods
        %% Constructor
        function obj = Trajectory()
            obj.storedPositions = zeros(3, obj.Length); % Initialize with zeros
        end

        %% Plotting
        function initTrajectoryPlot(obj, currentPosition)
            obj.Index = 1;
            obj.storedPositions(:, :) = repmat(currentPosition, 1, obj.Length);
            obj.PlotHandle = plot3(obj.storedPositions(1, :), obj.storedPositions(2, :), obj.storedPositions(3, :), 'k-', 'LineWidth', 1);
        end

        % Method to update trajectory
        function updateTrajectoryPlot(obj, currentPosition)
            obj.storedPositions(:, obj.Index) = currentPosition;
            obj.Index = mod(obj.Index, obj.Length) + 1;
            rolledstoredPositions = circshift(obj.storedPositions, [0, -obj.Index + 1]);
            set(obj.PlotHandle, 'XData', rolledstoredPositions(1, :), 'YData', rolledstoredPositions(2, :), 'ZData', rolledstoredPositions(3, :));
        end
    end
end
