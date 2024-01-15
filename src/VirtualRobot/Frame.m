classdef Frame < handle

    properties(Constant)
        AXES_COLORS = ['r', 'g', 'b'];
        AXIS_LABELS = {'X', 'Y', 'Z'};
    end

    properties
        p_r_f               % Position vector from the parent frames origin to this frames origin in the parent frames coordinate system
        p_A_f               % Rotation Matrix transforming from the frame to the parent frame
        rotationAxisLabel   % String label for the allowed local rotation axis
        parent              % Reference to the parent Frame object
        children            % Array of references to the child Frame objects

        % Plotting
        label               % String label for the frame
        axisHandles       % Graphics handles for X, Y, Z axes
        axisTextHandles   % Graphics handles for X, Y, Z axis labels
        textHandle        % Graphics handle for the frame label
    end

    methods
        %% Constructor
       function obj = Frame(p_r_f, parent, label, rotationAxisLabel)

            obj.p_r_f = p_r_f;

            obj.p_A_f = eye(3); % Upon instantiation, the frames rotation is the same as the parent frames rotation

            obj.label = label; 

            % The origin frame (frame without a parent) must not have a
            % rotation axis
            if isempty(parent) && ~isempty(rotationAxisLabel)
                error('Origin Frame must not have a rotation axis.')
            end

            % Setup Parent Child relationship
            obj.parent = parent; % Register the parent to this frame
            if ~isempty(parent)
                parent.children = [parent.children; obj]; % Register this frame to the parent frames children
            end
            obj.children = [];
            
            % Initialize graphics handle arrays with empty handles
            obj.axisHandles = [gobjects(1,1), gobjects(1,1), gobjects(1,1)];
            obj.axisTextHandles = [gobjects(1,1), gobjects(1,1), gobjects(1,1)];
            
            % Validate rotationAxisLabel
            if isempty(rotationAxisLabel) || any(strcmp(rotationAxisLabel, {'x', 'y', 'z'}))
                obj.rotationAxisLabel = rotationAxisLabel;
            else
                error('CustomFrame: rotationAxisLabel must be ''x'', ''y'', ''z'', or []');
            end   
        end
        
        %% Calculation
        function updateRotationMatrix(obj, angle)
            
            % Set the rotation of the Frame Around Its Local Axis

            switch lower(obj.rotationAxisLabel)
                case 'x'
                    obj.p_A_f = Frame.rotx(angle);
                case 'y'
                    obj.p_A_f = Frame.roty(angle);
                case 'z'
                    obj.p_A_f = Frame.rotz(angle);
                otherwise
                    error('Error setting angle for frame: %s\ninvalid rotation axis label', obj.label);
            end
            
        end

        function g_r_f = getGlobalPosition(obj)

            % Calculate the position vector from the global origin to this
            % frames origin in the global frames coordinate system

            currentFrame = obj;
            g_r_f = currentFrame.p_r_f; % Initialize with position relative to parent
        
            % Iterate backwards through the kinematic chain
            while ~isempty(currentFrame.parent)
                % Apply the parent's rotation matrix to the current position vector
                g_r_f = currentFrame.parent.p_A_f * g_r_f;
                
                % Add the parent's position vector to the current global position vector
                g_r_f = g_r_f + currentFrame.parent.p_r_f;
                
                % Move to the next parent in the hierarchy
                currentFrame = currentFrame.parent;
            end
        end


        function g_A_f = getGlobalRotationMatrix(obj)

            % Calculate the rotation matrix transforming from this frame to the
            % global frame

            currentFrame = obj;
            g_A_f = currentFrame.p_A_f; % Initialize with the frame's own rotation matrix
        
            % Iterate backwards through the kinematic chain
            while ~isempty(currentFrame.parent)
                % Multiply the accumulated rotation matrix with the parent's rotation matrix
                g_A_f = currentFrame.parent.p_A_f * g_A_f;
        
                % Move to the next parent in the hierarchy
                currentFrame = currentFrame.parent;
            end
        end
 
        %% Plotting Methods

        function initFramePlot(obj)
            scale_factor = 50;
            g_r_f = obj.getGlobalPosition;
            g_A_f = obj.getGlobalRotationMatrix;
            
            for i = 1:3
                endPos = g_r_f + scale_factor * g_A_f(:, i);
                
                % Initialize the axis line
                obj.axisHandles(i) = plot3([g_r_f(1), endPos(1)], ...
                                           [g_r_f(2), endPos(2)], ...
                                           [g_r_f(3), endPos(3)], ...
                                           'Color', obj.AXES_COLORS(i), 'LineWidth', 2);
        
                % Initialize the axis label
                obj.axisTextHandles(i) = text(endPos(1), endPos(2), endPos(3), obj.AXIS_LABELS{i}, ...
                                              'Color', obj.AXES_COLORS(i), 'FontWeight', 'bold');
            end
        
            % Initialize frame label
            % Uncomment and implement if frame label is needed
            % labelPos = g_r_f;
            % obj.textHandle = text(labelPos(1), labelPos(2), labelPos(3), obj.label, 'Color', 'k', 'FontWeight', 'bold');
        end

        function updateFramePlot(obj)
            scale_factor = 50;
            g_r_f = obj.getGlobalPosition;
            g_A_f = obj.getGlobalRotationMatrix;
        
            for i = 1:3
                endPos = g_r_f + scale_factor * g_A_f(:, i);
        
                % Update the axis line
                obj.axisHandles(i).XData = [g_r_f(1), endPos(1)];
                obj.axisHandles(i).YData = [g_r_f(2), endPos(2)];
                obj.axisHandles(i).ZData = [g_r_f(3), endPos(3)];
        
                % Update the axis label
                obj.axisTextHandles(i).Position = endPos;
        
                % Update the frame label if needed
                % Uncomment and implement if frame label is being used
                % obj.textHandle.Position = g_r_f;
            end
        end

    end

    methods(Static)
    
        %% Definition of the basis rotation matrices (See Wikipedia 'Rotation Matrix')
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
