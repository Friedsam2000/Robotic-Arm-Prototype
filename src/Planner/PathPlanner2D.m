classdef PathPlanner2D < handle
    % The planner slices the workspace of the robot at a given height and
    % lets the user draw waypoints. It then returns these waypoints (the
    % path) as a waypoint list. The first waypoint is the current
    % endeffector position of the virtual robot

    properties (Access = private)
        virtualRobot  % Referenced instance of VirtualRobot
        height        % Given height z for the 2D slice
        pathX         % X coordinates of the drawn path
        pathY         % Y coordinates of the drawn path
        pathZ         % Z coordinate of the drawn path
        segments      % Segments of the workspace at the given height z
        hFig
    end
    
    methods
        function obj = PathPlanner2D(virtualRobot, z)
            % Constructor
            obj.virtualRobot = virtualRobot;
            obj.height = z;
            obj.segments = obj.calculateWorkspaceSlice;
        end
        
        function waypoint_list = getWaypointList(obj)
            % Returns the X, Y, and Z coordinates of the drawn path
            if isempty(obj.pathX) || isempty(obj.pathY) || isempty(obj.pathZ)
                warning('Path has not been defined. Run userInputPath first.');
                waypoint_list = [];
                return;
            end
            x = obj.pathX';
            y = obj.pathY';
            z = obj.pathZ';

            waypoint_list = [x;y; z];

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
            currentX = currentPosition(1);
            currentY = currentPosition(2);
        
            % Set the starting point of the path to directly below the current endeffector position
            obj.pathX = [currentX];
            obj.pathY = [currentY];
            obj.pathZ = [obj.height];
        
            % Plot the starting point on the figure
            plot(currentX, currentY, 'o', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
        
            % Display instructions on the figure
            instructionText = 'Draw the path by adding waypoints with left-click. Finish with right-click.';
            annotation('textbox', [0.15, 0.8, 0.7, 0.1], 'String', instructionText, 'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'FontSize', 10);

        
            % Use a while loop to continuously capture points until right-click
            while true
                [xi, yi, button] = ginput(1);
            
                % If the user pressed right-click without having set any points, display a warning.
                if button == 3 && length(obj.pathX) < 2
                    disp("Create at least one waypoint using leftclick!");
                    continue;  % Skip the rest of the loop iteration
                end
            
                % Break the loop if right-clicked (button value is 3 for right-click)
                if button == 3
                    break;
                end
            
                % Add the point to the path arrays
                if button == 1 && (isempty(obj.pathX) || (xi ~= obj.pathX(end) || yi ~= obj.pathY(end)))  % Check if left-click and not a duplicate
                    obj.pathX = [obj.pathX; xi];
                    obj.pathY = [obj.pathY; yi];
                    obj.pathZ = [obj.pathZ; obj.height];
                            
                    % Plot the point immediately
                    plot(xi, yi, 'o', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
                                    
                    % If there's more than one point, plot a line segment connecting the last two points
                    if length(obj.pathX) > 1
                        plot(obj.pathX(end-1:end), obj.pathY(end-1:end), 'b-');
                    end
                end
            end
            close(obj.hFig);
        end
        
    end

    methods (Access = private)
        function segments = calculateWorkspaceSlice(obj)
            % Calculate the intersection of the robot's workspace with the plane defined by z

            segments = {};
            for i = 1:size(obj.virtualRobot.workspace.boundaryTriangulation, 1)
                tri = obj.virtualRobot.workspace.uniqueWorkspaceSamples(obj.virtualRobot.workspace.boundaryTriangulation(i, :), :);  % Extract triangle vertices
                
                % Check if the triangle intersects the plane
                isAbove = tri(:, 3) > obj.height;
                isBelow = tri(:, 3) < obj.height;
                if any(isAbove) && any(isBelow)  % At least one vertex is above and one is below
                    % Triangle intersects the plane
                    
                    % Compute the two intersection points with the plane
                    intersectedLine = obj.computeIntersection(tri);
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
