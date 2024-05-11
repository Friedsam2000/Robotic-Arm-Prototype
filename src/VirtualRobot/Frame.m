classdef Frame < handle

    properties(Constant)
        AXES_COLORS = ['r', 'g', 'b'];
        AXIS_LABELS = {'X', 'Y', 'Z'};
    end

    properties
        relativePosition = [];               % (p_r_pf) Position vector from the parent frames origin to this frames origin in the parent frames coordinate system
        relativeRotation = [];               % (relativeRotation) Rotation Matrix transforming from the frame to the parent frame 
        rotationAxis = [];   % Char label for the allowed local rotation axis
        parent = [];              % Reference to the parent Frame object

        % Plotting
        label = [];               % String label for the frame
        axisHandles = [];       % Graphics handles for X, Y, Z axes
        axisTextHandles = [];   % Graphics handles for X, Y, Z axis labels
        textHandle = [];        % Graphics handle for the frame label
    end

    methods
        %% Constructor
       function obj = Frame(relativePosition, parent, label, rotationAxis)

            obj.relativePosition = relativePosition;

            obj.relativeRotation = eye(3); % Upon instantiation, the frames rotation is the same as the parent frames rotation

            obj.label = label; 

            % Setup Parent Child relationship
            obj.parent = parent; % Register the parent to this frame
            
            % Initialize graphics handle arrays with empty handles
            obj.axisHandles = [gobjects(1,1), gobjects(1,1), gobjects(1,1)];
            obj.axisTextHandles = [gobjects(1,1), gobjects(1,1), gobjects(1,1)];
            
            % Validate rotationAxis
            if isempty(rotationAxis) || any(strcmp(rotationAxis, {'x', 'y', 'z'}))
                obj.rotationAxis = rotationAxis;
            else
                error('CustomFrame: rotationAxis must be ''x'', ''y'', ''z'', or []');
            end   
        end
        
        %% Calculation
        function updateRotationMatrix(obj, angle)
            
            % Set the rotation of the Frame Around Its Local Axis

            switch lower(obj.rotationAxis)
                case 'x'
                    obj.relativeRotation = Frame.rotx(angle);
                case 'y'
                    obj.relativeRotation = Frame.roty(angle);
                case 'z'
                    obj.relativeRotation = Frame.rotz(angle);
                otherwise
                    error('Error setting angle for frame: %s\ninvalid rotation axis label', obj.label);
            end
            
        end

        function globalPosition = getGlobalPosition(obj)

            % (g_r_gf) pCalculate the position vector from the global origin to this
            % frames origin in the global frames coordinate system

            currentFrame = obj;
            globalPosition = currentFrame.relativePosition; % Initialize with position relative to parent
        
            % Iterate backwards through the kinematic chain
            while ~isempty(currentFrame.parent)
                % Apply the parent's rotation matrix to the current position vector
                globalPosition = currentFrame.parent.relativeRotation * globalPosition;
                
                % Add the parent's position vector to the current global position vector
                globalPosition = globalPosition + currentFrame.parent.relativePosition;
                
                % Move to the next parent in the hierarchy
                currentFrame = currentFrame.parent;
            end
        end

        function globalRotation = getGlobalRotation(obj)

            % (g_R_f) Calculate the rotation matrix transforming from this frame to the
            % global frame

            currentFrame = obj;
            globalRotation = currentFrame.relativeRotation; % Initialize with the frame's own rotation matrix
        
            % Iterate backwards through the kinematic chain
            while ~isempty(currentFrame.parent)
                % Multiply the accumulated rotation matrix with the parent's rotation matrix
                globalRotation = currentFrame.parent.relativeRotation * globalRotation;
        
                % Move to the next parent in the hierarchy
                currentFrame = currentFrame.parent;
            end
        end
 
        %% Plotting Methods

        function initFramePlot(obj)
            scale_factor = 50;
            globalPosition = obj.getGlobalPosition;
            globalRotation = obj.getGlobalRotation;
            
            for i = 1:3
                endPos = globalPosition + scale_factor * globalRotation(:, i);
                
                % Initialize the axis line
                obj.axisHandles(i) = plot3([globalPosition(1), endPos(1)], ...
                                           [globalPosition(2), endPos(2)], ...
                                           [globalPosition(3), endPos(3)], ...
                                           'Color', obj.AXES_COLORS(i), 'LineWidth', 2);
        
                % Initialize the axis label
                obj.axisTextHandles(i) = text(endPos(1), endPos(2), endPos(3), obj.AXIS_LABELS{i}, ...
                                              'Color', obj.AXES_COLORS(i), 'FontWeight', 'bold');
            end
        
            % Initialize frame label
            % Uncomment and implement if frame label is needed
            % labelPos = globalPosition;
            % obj.textHandle = text(labelPos(1), labelPos(2), labelPos(3), obj.label, 'Color', 'k', 'FontWeight', 'bold');
        end

        function updateFramePlot(obj)
            scale_factor = 50;
            globalPosition = obj.getGlobalPosition;
            globalRotation = obj.getGlobalRotation;
        
            for i = 1:3
                endPos = globalPosition + scale_factor * globalRotation(:, i);
        
                % Update the axis line
                obj.axisHandles(i).XData = [globalPosition(1), endPos(1)];
                obj.axisHandles(i).YData = [globalPosition(2), endPos(2)];
                obj.axisHandles(i).ZData = [globalPosition(3), endPos(3)];
        
                % Update the axis label
                obj.axisTextHandles(i).Position = endPos;
        
                % Update the frame label if needed
                % Uncomment and implement if frame label is being used
                % obj.textHandle.Position = globalPosition;
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
