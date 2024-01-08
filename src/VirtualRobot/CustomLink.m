classdef CustomLink < handle
    properties
        startFrame   % Starting frame of the link
        endFrame     % Ending frame of the link
        lineHandle   % Graphics handle for the link
        color        % Link color
    end
    
    methods
       function obj = CustomLink(startFrame, endFrame, color)
            obj.startFrame = startFrame;
            obj.endFrame = endFrame;
            obj.color = color;
            obj.lineHandle = [];  % Initialize line handle as empty
       end

       function newObj = copy(obj)
            % Assume startFrame and endFrame are already copied and passed as arguments
            newObj = CustomLink([], [], obj.color);
            % Note: You need to set startFrame and endFrame later in the VirtualRobot copy method
       end
        
        
        function initLinkPlot(obj)
            % Get start and end positions
            startPos = obj.startFrame.getGlobalPosition;
            endPos = obj.endFrame.getGlobalPosition;
            obj.lineHandle = plot3([startPos(1), endPos(1)], ...
                [startPos(2), endPos(2)], ...
                [startPos(3), endPos(3)], '-', ...
                'Color', obj.color, 'LineWidth', 5);
        end

        function updateLinkPlot(obj)
            % Get start and end positions
            startPos = obj.startFrame.getGlobalPosition;
            endPos = obj.endFrame.getGlobalPosition;
            set(obj.lineHandle, 'XData', [startPos(1), endPos(1)], ...
                'YData', [startPos(2), endPos(2)], ...
                'ZData', [startPos(3), endPos(3)]);
        end

    end
end
