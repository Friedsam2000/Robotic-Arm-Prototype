classdef Workspace < handle

    properties (Constant)
        workspaceResolution = 0.1; % Angular resolution for workspace sampling in radians.
        workspaceTolerance = 0.02; % Tolerance for filtering near-identical workspace samples.
    end

    properties
        virtualRobot; % Reference to the virtualRobot object for workspace sampling and plotting.

        surfaceMesh;       % Matrix (3 x m) representing the triangulated surface of the workspace.
        % Each column contains 3 indices which refer to
        % the columns (points) in workspacePoints that
        % form a triangle. These triangles compose
        % a surface mesh.

        workspacePoints;   % Matrix (3 x n) of distinct workspace sample points.
        % Each column is a sample point of the workspace.
    end

    methods
        %% Constructor
        function obj = Workspace(virtualRobot)

            obj.virtualRobot = virtualRobot;

            % Load workspace data from file, or calculate if file not found or joint limits changed
            if exist('workspaceObject.mat', 'file')
                loadedData = load('workspaceObject.mat');
                if isequal(loadedData.obj.virtualRobot.JOINT_ANGLE_LIMITS, obj.virtualRobot.JOINT_ANGLE_LIMITS)
                    % Use loaded workspace data if joint limits are unchanged
                    obj.surfaceMesh = loadedData.obj.surfaceMesh;
                    obj.workspacePoints = loadedData.obj.workspacePoints;
                else
                    % Recalculate workspace for new joint limits
                    obj.calculateWorkspace;
                end
            else
                % Calculate workspace when no saved data is available
                obj.calculateWorkspace;
            end
        end

        %% Calculation
        function calculateWorkspace(obj)

            % Calculates the workspace of the robot by sampling.
            tic;
            disp("Calculating Workspace...");

            % Get joint limits from the virtual robot
            JOINT_ANGLE_LIMITS = obj.virtualRobot.JOINT_ANGLE_LIMITS;

            % Define joint angle ranges based on limits and resolution
            q_1 = JOINT_ANGLE_LIMITS(1, 1):obj.workspaceResolution:JOINT_ANGLE_LIMITS(1, 2);
            q_2 = JOINT_ANGLE_LIMITS(2, 1):obj.workspaceResolution:JOINT_ANGLE_LIMITS(2, 2);
            q_3 = JOINT_ANGLE_LIMITS(3, 1):obj.workspaceResolution:JOINT_ANGLE_LIMITS(3, 2);
            q_4 = JOINT_ANGLE_LIMITS(4, 1):obj.workspaceResolution:JOINT_ANGLE_LIMITS(4, 2);

            % Initialize workspace array
            num_samples = length(q_1) * length(q_2) * length(q_3) * length(q_4);
            workspaceSamples = zeros(num_samples, 3);
            n = 1;

            % Store the current joint configuration of the referenced
            % virtual robot object
            q_save = obj.virtualRobot.getJointAngles;

            % Nested loops to iterate through all combinations of joint angles
            for i = 1:length(q_1)
                for j = 1:length(q_2)
                    for k = 1:length(q_3)
                        for l = 1:length(q_4)
                            % Set the robot's joint angles
                            obj.virtualRobot.setJointAngles([q_1(i); q_2(j); q_3(k); q_4(l)]);

                            % Calculate and store the end-effector position
                            workspaceSamples(n, :) = obj.virtualRobot.forwardKinematics();
                            n = n + 1;
                        end
                    end
                end
            end

            % Restore the configuration of the virtual robot object
            obj.virtualRobot.setJointAngles(q_save);
            % Reduce the number of workspace points using uniquetol
            obj.workspacePoints = uniquetol(workspaceSamples, obj.workspaceTolerance, 'ByRows', true).';

            % Compute a 3D boundary of the robot's reduced workspace
            obj.surfaceMesh = boundary(obj.workspacePoints(1, :)', obj.workspacePoints(2, :)', obj.workspacePoints(3, :)').';

            fprintf("Serial Workspace Calculation took %.2f seconds\n" + ...
                "Unique Workspace Samples: %d \n\n", toc, size(obj.workspacePoints, 2));

            % Save the workspace object to VirtualRobot folder
            % Get the directory of the currently executing script
            currentFile = mfilename('fullpath');
            [currentDir, ~, ~] = fileparts(currentFile);

            % Construct the paths to the folders one level up
            parentDir = fullfile(currentDir, '..');  % This goes one level up to src
            virtualRobotDir = fullfile(parentDir, 'VirtualRobot');

            % Specify the full path for the .mat file to be saved
            matFilePath = fullfile(virtualRobotDir, 'workspaceObject.mat');

            % Save the workspace object 'obj' to the specified path
            save(matFilePath, 'obj');
        end

        %% Plotting
        function initWorkspacePlot(obj)
            % Plot the boundary using trisurf
            workspacePlot = trisurf(obj.surfaceMesh', obj.workspacePoints(1, :), ...
                obj.workspacePoints(2, :), obj.workspacePoints(3, :), ...
                'Facecolor', 'cyan', 'Edgecolor', 'none');
            light('Position', [1 3 2]);
            lighting gouraud;
            alpha(workspacePlot, 0.1);  % Make it slightly transparent
        end
    end
end
