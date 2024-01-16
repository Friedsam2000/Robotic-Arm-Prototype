classdef PathPlanner < handle
    properties
        
        path          % col : point
        virtualRobot  % Referenced instance of VirtualRobot

        height        % Given height z for the 2D slice
        segments      % Segments of the workspace at the given height z
        hFig
    end

    methods
        function obj = PathPlanner(virtualRobot, z)
            % Constructor
            obj.virtualRobot = virtualRobot;
            obj.height = z;
            obj.segments = obj.calculateWorkspaceSlice;
            obj.path = []; % Initialize the path matrix
            obj.userInputPath;
        end

        function userInputPath(obj)
            % Open a new figure window with grid on and specified limits
            obj.hFig = figure;

            % Determine axis limits based on the workspace slice
            allXs = cell2mat(cellfun(@(seg) seg(:, 1), obj.segments, 'UniformOutput', false));
            allYs = cell2mat(cellfun(@(seg) seg(:, 2), obj.segments, 'UniformOutput', false));

            margin = 100;
            minX = min(min(allXs)) - margin;
            maxX = max(max(allXs)) + margin;
            minY = min(min(allYs)) - margin;
            maxY = max(max(allYs)) + margin;

            axis([minX maxX minY maxY]);
            grid on;
            hold on;
            xlabel('X');
            ylabel('Y');
            title('Trajectory in 2D Plane');

            % Plot the workspace segments
            for i = 1:length(obj.segments)
                seg = obj.segments{i};
                plot(seg(:, 1), seg(:, 2), 'g-', 'LineWidth', 2);
            end

            % Get current virtualRobot position in the XY-plane
            currentPosition = obj.virtualRobot.forwardKinematics;

            % Set the starting point of the path to directly below the current endeffector position
            obj.path = [currentPosition(1); currentPosition(2); obj.height];

            % Plot the starting point on the figure
            plot(currentPosition(1), currentPosition(2), 'o', 'MarkerSize', 6, 'MarkerFaceColor', 'r');

            % Display instructions on the figure
            instructionText = 'Draw the path by adding waypoints with left-click. Finish with right-click.';
            annotation('textbox', [0.15, 0.8, 0.7, 0.1], 'String', instructionText, 'EdgeColor', 'none', 'HorizontalAlignment', 'center','FontSize', 10);

            % Use a while loop to continuously capture points until right-click
            while true
                [xi, yi, button] = ginput(1);

                % If the user pressed right-click without having set any points, display a warning.
                if button == 3 && size(obj.path,1) < 2
                    disp("Create at least one waypoint using left-click!");
                    continue;  % Skip the rest of the loop iteration
                end

                % Break the loop if right-clicked (button value is 3 for right-click)
                if button == 3
                    break;
                end

                % Add the point to the path matrix
                if button == 1 && (isempty(obj.path) || (xi ~= obj.path(end, 1) || yi ~= obj.path(end, 2)))  % Check if left-click and not a duplicate
                    obj.path = [obj.path, [xi; yi; obj.height]];

                    % Plot the point immediately
                    plot(xi, yi, 'o', 'MarkerSize', 6, 'MarkerFaceColor', 'r');

                    % If there's more than one point, plot a line segment connecting the last two points
                    if size(obj.path, 1) > 1
                        plot(obj.path(1,end-1:end), obj.path(2,end-1:end), 'b-');
                    end
                end
            end
            close(obj.hFig);
        end


        %% Plotting
        function segments = calculateWorkspaceSlice(obj)
            % Calculate the intersection of the robot's workspace with the plane defined by z

            segments = {};
            for i = 1:size(obj.virtualRobot.workspace.surfaceMesh, 2)
                tri = obj.virtualRobot.workspace.workspacePoints(:,obj.virtualRobot.workspace.surfaceMesh(:,i));  % Extract triangle vertices

                % Check if the triangle intersects the plane
                isAbove = tri(3,:) > obj.height;
                isBelow = tri(3,:) < obj.height;
                if any(isAbove) && any(isBelow)  % At least one vertex is above and one is below
                    % Triangle intersects the plane

                    % Compute the two intersection points with the plane
                    intersectedLine = obj.computeIntersection(tri');
                    if ~isempty(intersectedLine)
                        segments{end+1} = intersectedLine;
                    end
                end
            end

            if isempty(segments)
                warning('The specified plane does not intersect the workspace.');
            end
        end

        function intersectedLine = computeIntersection(obj, tri)
            % Computes the intersection lines of a triangle with a plane at height z
            intersectedPts = [];

            for i = 1:3
                p1 = tri(i, :);
                p2 = tri(mod(i, 3) + 1, :);  % next vertex in the triangle

                if (p1(3) < obj.height && p2(3) > obj.height) || (p1(3) > obj.height && p2(3) < obj.height)
                    alpha = (obj.height - p1(3)) / (p2(3) - p1(3));  % Compute intersection ratio
                    intersectionPt = p1 + alpha * (p2 - p1);  % Intersection point in 3D
                    intersectedPts = [intersectedPts; intersectionPt(1:2)];  % Store only X and Y
                end
            end
            if size(intersectedPts, 1) == 2
                intersectedLine = intersectedPts;
            else
                intersectedLine = [];
            end
        end
    end
end