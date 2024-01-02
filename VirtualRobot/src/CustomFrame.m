classdef CustomFrame < handle

    properties (SetAccess = private)
        p_r_f               % Position vector from the parent frames origin to this frames origin in the parent frames coordinate system
        p_A_f               % Rotation Matrix transforming from the frame to the parent frame
        label               % String label for the frame
        rotationAxisLabel   % String label for the allowed local rotation axis
    end

    properties
        parent              % Reference to the parent Frame object
        children            % Array of references to the child Frame objects
    end

    properties (Access = private, Hidden)
        % Graphics handles in arrays
        axisHandles       % Graphics handles for X, Y, Z axes
        axisTextHandles   % Graphics handles for X, Y, Z axis labels
        textHandle        % Graphics handle for the frame label
    end

    properties(Constant, Hidden)
        AXES_COLORS = ['r', 'g', 'b'];
        AXIS_LABELS = {'X', 'Y', 'Z'};
    end

    methods
        function obj = CustomFrame(p_r_f, parent, label, rotationAxisLabel)

            obj.p_r_f = p_r_f;

            obj.p_A_f = eye(3); % Upon instantiation, the frames orientation is the same as the parent frames orientation

            obj.parent = parent; % Register the parent to this frame
            obj.label = label;

            if ~isempty(parent)
                parent.children = [parent.children; obj]; % Add this frame to the parent frames children
            end
            
            obj.children = [];
            
            % Initialize graphics handle arrays with empty handles
            obj.axisHandles = [gobjects(1,1), gobjects(1,1), gobjects(1,1)];
            obj.axisTextHandles = [gobjects(1,1), gobjects(1,1), gobjects(1,1)];
            
            % Validate rotationAxisLabel
            if isempty(rotationAxisLabel) || any(strcmp(rotationAxisLabel, {'x', 'y', 'z'}))
                obj.rotationAxisLabel = rotationAxisLabel;
            else
                error('rotationAxisLabel must be ''x'', ''y'', ''z'', or []');
            end   
        end
        
       function newObj = copy(obj)
            % Creating a new instance with shallow copy
            newObj = CustomFrame(obj.p_r_f, [], obj.label, obj.rotationAxisLabel);
            % Do not copy children here. It will be handled in VirtualRobot's copy method.
        end

        % Calculate the position vector from the global origin to this
        % frames origin in the global frames coordinate system
        function g_r_f = getGlobalPosition(obj)
            currentFrame = obj;
            g_r_f = currentFrame.p_r_f; % Initialize with position relative to parent
        
            while ~isempty(currentFrame.parent)
                % Apply the parent's rotation matrix to the current position vector
                g_r_f = currentFrame.parent.p_A_f * g_r_f;
                
                % Add the parent's position vector to the current global position vector
                g_r_f = g_r_f + currentFrame.parent.p_r_f;
                
                % Move to the next parent in the hierarchy
                currentFrame = currentFrame.parent;
            end
        end

        % Calculate the rotation matrix transforming from this frame to the
        % global frame
        function g_A_f = getGlobalRotationMatrix(obj)
            currentFrame = obj;
            g_A_f = currentFrame.p_A_f; % Initialize with the frame's own rotation matrix
        
            while ~isempty(currentFrame.parent)
                % Multiply the accumulated rotation matrix with the parent's rotation matrix
                g_A_f = currentFrame.parent.p_A_f * g_A_f;
        
                % Move to the next parent in the hierarchy
                currentFrame = currentFrame.parent;
            end
        end

        
        % Set the rotation of the Frame Around Its Local Axis
        function setAngle(obj, angle)
            
            % Update the rotation matrix relative to the frames parent
            % p_A_f
            switch lower(obj.rotationAxisLabel)
                case 'x'
                    obj.p_A_f = CustomFrame.rotx(angle);
                case 'y'
                    obj.p_A_f = CustomFrame.roty(angle);
                case 'z'
                    obj.p_A_f = CustomFrame.rotz(angle);
                otherwise
                    error("Error setting angle for frame: %s \n" + ...
                        "invalid rotation axis label \n", obj.label);
            end
            
        end

        % Draw or update the frame
        function draw(obj)
            scale_factor = 50;
            g_r_f = obj.getGlobalPosition;
            g_A_f = obj.getGlobalRotationMatrix;
            
            for i = 1:3
                endPos = g_r_f + scale_factor * g_A_f(:, i);
                
                % Draw or update the axis line
                obj.axisHandles(i) = CustomFrame.drawLineOrRefresh(obj.axisHandles(i), g_r_f, endPos, obj.AXES_COLORS(i));
    
                % Draw or update the axis label
                obj.axisTextHandles(i) = CustomFrame.drawTextOrRefresh(obj.axisTextHandles(i), endPos, obj.AXIS_LABELS{i}, obj.AXES_COLORS(i));

                 % Draw or update the frame label
                % labelPos = g_r_f;
                % obj.textHandle = CustomFrame.drawTextOrRefresh(obj.textHandle, labelPos, obj.label, 'k'); % 'k' for black color, or choose another
            

            end
        end
    end

    methods(Static, Access = private, Hidden)
        function handle = drawLineOrRefresh(handle, startPos, endPos, color)
            if isempty(handle) || ~isvalid(handle) || isa(handle, 'matlab.graphics.GraphicsPlaceholder')
                handle = plot3([startPos(1), endPos(1)], ...
                               [startPos(2), endPos(2)], ...
                               [startPos(3), endPos(3)], ...
                               'Color', color, 'LineWidth', 2);
            else
                handle.XData = [startPos(1), endPos(1)];
                handle.YData = [startPos(2), endPos(2)];
                handle.ZData = [startPos(3), endPos(3)];
            end
        end

        function handle = drawTextOrRefresh(handle, position, label, color)
            if isempty(handle) || ~isvalid(handle) || isa(handle, 'matlab.graphics.GraphicsPlaceholder')
                handle = text(position(1), position(2), position(3), label, 'Color', color, 'FontWeight', 'bold');
            else
                % Only set the position if handle is not a placeholder
                if ~isa(handle, 'matlab.graphics.GraphicsPlaceholder')
                    handle.Position = position;
                end
            end
        end

    end

    methods(Static)
    
        %% Definition of the basis rotation matrices
        function rotx = rotx(alpha)
            rotx = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
        end
        
        function roty = roty(beta)
            roty = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
        end
        
        function rotz = rotz(gamma)
            rotz = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];
        end
    end

end
