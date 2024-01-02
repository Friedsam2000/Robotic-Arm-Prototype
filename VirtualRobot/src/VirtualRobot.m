classdef VirtualRobot < handle

    properties (SetAccess = private, Hidden)
        links   % An array of Link objects, defining the physical connections between joints
        frames % An array of Frame objects, defining joints and frames
        fig % The figure in which everything is visualized

        joint_limits = [-pi/4, pi/4;
                        -pi/4, pi/4; 
                        -2*pi, 2*pi; 
                        -(5/6)*pi, (5/6)*pi];

        q = [0;0;0;0]; % Joint Angles
        % Workspace
        workspaceResolution = 0.13;
        workspaceTolerance = 0.02;
        boundaryK % A property to store the computed 3D boundary of the workspace
        boundaryVertices % A property to store the vertices of the boundary
    end

    methods (Access = public)
        
        function obj = VirtualRobot(varargin)
            
            % Set default values for optional arguments
            calc_workspace = 1;
        
            % Check if optional arguments were provided and update accordingly
            if length(varargin) >= 1
                calc_workspace = varargin{1};
            end

            %% Setup Frames and Joints of the simulated robot
            orig_frame = CustomFrame([0; 0; 0], [], 'Origin', []);
            joint1 = CustomFrame([0; 0; 94.5127], orig_frame, 'Joint 1', 'y');
            joint2 = CustomFrame([0;0;0], joint1, 'Joint 2', 'x');
            joint3 = CustomFrame([0;0;150.0277], joint2, 'Joint 3', 'z');
            joint4 = CustomFrame([0;0;157.8995], joint3, 'Joint 4', 'x');
            endeffector_frame = CustomFrame([0;0;220.40], joint4, 'Endeffector', []);
            
            %% Setup Links of the simulated robot
            numLinks = 5; % Number of links
            grayLevels = linspace(0.2, 0.8, numLinks);  % Define a range of grayscale values. Start from 0.2 (dark) to 0.8 (lighter)
            
            link1 = CustomLink(orig_frame, joint1, repmat(grayLevels(1), 1, 3));  % Link 1 with first grayscale value
            link2 = CustomLink(joint1, joint2, repmat(grayLevels(2), 1, 3));  % Link 2 with second grayscale value
            link3 = CustomLink(joint2, joint3, repmat(grayLevels(3), 1, 3));  % and so on...
            link4 = CustomLink(joint3, joint4, repmat(grayLevels(4), 1, 3));
            link5 = CustomLink(joint4, endeffector_frame, repmat(grayLevels(5), 1, 3));

            obj.links = [link1, link2, link3, link4, link5];
            obj.frames = [orig_frame, joint1, joint2, joint3, joint4, endeffector_frame];

            % calculate workspace
            if calc_workspace
                obj.calculateWorkspaceParallel;
                %  Alternatively use
                 % obj.calculateWorkspaceSerial;
            end

        end

        function q = getQ(obj)
            q = obj.q;
        end

        function setQ(obj, q_desired)

            q_desired = mod(q_desired + 2*pi, 4*pi) - 2*pi;  % Convert the desired q to the range [-2pi, 2pi]
            for i = 1:4
                obj.frames(i+1).setAngle(q_desired(i));
                obj.q(i) = q_desired(i);
            end

        end

        function draw(obj, varargin)
            % The display method updates and displays all joints and links
            % Activates hold on, no hold off
            obj.ensureFigureExists();

             % Set default values for optional arguments
            draw_frames = 0;
        
            % Check if optional arguments were provided and update accordingly
            if length(varargin) >= 1
                draw_frames = varargin{1};
            end
            
            if draw_frames
                for i = 1:length(obj.frames)
                    obj.frames(i).draw
                end
            end

            for i = 1:length(obj.links)
                obj.links(i).draw
            end

            
        end

        function visualizeWorkspace(obj)
            % Visualize the 3D boundary of the robot's workspace

            obj.ensureFigureExists(); % Ensure figure exists before plotting
        
            % Plot the boundary
            trisurf(obj.boundaryK, obj.boundaryVertices(:, 1), obj.boundaryVertices(:, 2), obj.boundaryVertices(:, 3), 'Facecolor', 'cyan', 'Edgecolor', 'none');
            light('Position', [1 3 2]);
            lighting gouraud
            alpha 0.1  % Make it slightly transparent
        end
   
        function [g_r_EE] = getEndeffectorPos(obj)
            g_r_EE = obj.frames(end).getGlobalPosition;
        end

        function [elevation] = getShoulderElevation(obj)
            % Calculate the elevation of the shoulder joint in spherical
            % coordinates in RAD from q
            q = obj.getQ;
            elevation = pi/2 - acos(cos(q(2))*cos(q(1)));           
        end

    end

    methods (Access = public, Hidden)

        function newObj = copy(obj)
            newObj = VirtualRobot(0); % Create a new VirtualRobot instance
    
            % Step 1: Deep copy of frames
            for i = 1:length(obj.frames)
                newObj.frames(i) = obj.frames(i).copy();
            end
    
            % Step 2: Reconstruct parent-child relationships
            for i = 1:length(obj.frames)
                originalFrame = obj.frames(i);
                copiedFrame = newObj.frames(i);
    
                if ~isempty(originalFrame.parent)
                    parentIndex = find(obj.frames == originalFrame.parent);
                    copiedFrame.parent = newObj.frames(parentIndex);
                end
    
                for j = 1:length(originalFrame.children)
                    childIndex = find(obj.frames == originalFrame.children(j));
                    copiedFrame.children = [copiedFrame.children, newObj.frames(childIndex)];
                end
            end
    
            % Step 3: Deep copy of links
            for i = 1:length(obj.links)
                originalLink = obj.links(i);
                copiedLink = originalLink.copy(); % Assuming CustomLink has a copy method
    
                % Update the startFrame and endFrame of the copied link
                startFrameIndex = find(obj.frames == originalLink.startFrame);
                endFrameIndex = find(obj.frames == originalLink.endFrame);
                copiedLink.startFrame = newObj.frames(startFrameIndex);
                copiedLink.endFrame = newObj.frames(endFrameIndex);
    
                newObj.links(i) = copiedLink;
            end
        end

        function singularityBool = warnSingularity(obj)
            % Check for singularity
            J = obj.getJacobianNumeric;
            pinvJ = pinv(J);

            if norm(J)*norm(pinvJ) > 25
                disp('Warning: Close to singularity');
                singularityBool = true;
            else
                singularityBool = false;
            end
        end
        
        function J = getJacobianNumeric(obj)
            % Computes the Jacobian matrix numerically, relating joint velocities to end-effector velocities.
            % Uses finite differences on forward kinematics by perturbing joint angles with 'delta_q'. 
            % Numerical methods may offer speed advantages over symbolic ones, but precision can vary.
            
            % Small change in joint angles
            delta_q = 1e-6;
            
            % Initialize Jacobian matrix
            J = zeros(3, 4);

            q = obj.getQ;

            %Save q to restore it later
            q_save = q;
            
            % For each joint angle
            for i = 1:4
                % Perturb joint angle i
                q_plus = q;
                q_plus(i) = q_plus(i) + delta_q;
                q_minus = q;
                q_minus(i) = q_minus(i) - delta_q;
                
                % Compute forward kinematics for q_plus
                obj.setQ(q_plus)
                g_r_EE_plus = obj.getEndeffectorPos;
                
                % Compute forward kinematics for q_minus
                obj.setQ(q_minus)
                g_r_EE_minus = obj.getEndeffectorPos;
                
                % Compute derivative
                J(:, i) = (g_r_EE_plus - g_r_EE_minus) / (2 * delta_q);
            end
            % Restore original q
            obj.setQ(q_save);
        end

        function calculateWorkspaceParallel(obj, varargin)
            tic;
        
            joint_limits_local = obj.joint_limits;
        
            if ~isempty(obj.boundaryK) && nargin == 1
                disp("Workspace already calculated. Pass additional argument to update.");
                return;
            else
                disp("Calculating Workspace...");
            end
        
            % Define the range of joint angles
            q_1 = joint_limits_local(1, 1):obj.workspaceResolution:joint_limits_local(1, 2);
            q_2 = joint_limits_local(2, 1):obj.workspaceResolution:joint_limits_local(2, 2);
            q_3 = joint_limits_local(3, 1):obj.workspaceResolution:joint_limits_local(3, 2);
            q_4 = joint_limits_local(4, 1):obj.workspaceResolution:joint_limits_local(4, 2);
        
            % Ensure a parallel pool is open
            pool = gcp('nocreate'); % Get current parallel pool
            if isempty(pool)
                pool = parpool(4); % Create a new pool
            end
        
            % Calculate the number of combinations each worker needs to process
            total_combinations = length(q_1) * length(q_2) * length(q_3) * length(q_4);
            combinations_per_worker = ceil(total_combinations / pool.NumWorkers);
        
            % Use a cell array to store local workspaces
            localWorkspaces = cell(1, pool.NumWorkers);
        
            % Parallel loop
            parfor workerIdx = 1:pool.NumWorkers
                % Determine the range of combinations for this worker
                startIdx = (workerIdx - 1) * combinations_per_worker + 1;
                endIdx = min(workerIdx * combinations_per_worker, total_combinations);
        
                % Local workspace for this worker
                localWorkspace = zeros(3, endIdx - startIdx + 1);
        
                % Create a deep copy of the robot for each worker
                v_robot = obj.copy;
        
                localIdx = 1; % Local index for localWorkspace
                for idx = startIdx:endIdx
                    % Find the indices for each joint angle
                    [i, j, k, l] = ind2sub([length(q_1), length(q_2), length(q_3), length(q_4)], idx);
                    v_robot.setQ([q_1(i); q_2(j); q_3(k); q_4(l)]);
                    localWorkspace(:, localIdx) = v_robot.getEndeffectorPos;
                    localIdx = localIdx + 1;
                end
        
                % Store the local results in the cell array
                localWorkspaces{workerIdx} = localWorkspace;
            end

            % Close pool
            pool = gcp('nocreate'); % Get the handle to the current pool, if it exists
            if ~isempty(pool)
                delete(pool); % Close the pool
            end        

            % Concatenate the results from each worker
            workspace = horzcat(localWorkspaces{:});
        
            % Reduce the number of workspace points using uniquetol
            uniqueWorkspace = uniquetol(workspace', obj.workspaceTolerance, 'ByRows', true)';
        
            % Compute a 3D boundary (convex hull) of the robot's reduced workspace
            K = boundary(uniqueWorkspace(1, :)', uniqueWorkspace(2, :)', uniqueWorkspace(3, :)');
            v = uniqueWorkspace'; % The vertices of the boundary
        
            % Store the boundary
            obj.boundaryK = K;
            obj.boundaryVertices = v;
        
            fprintf("Parallel Workspace Calculation took %.2f seconds\n" + ...
                "BoundaryVertices Count: %d \n\n", toc, length(v));

        end

        function calculateWorkspaceSerial(obj, varargin)
            tic;
        
            joint_limits_local = obj.joint_limits;
        
            if ~isempty(obj.boundaryK) && nargin == 1
                disp("Workspace already calculated. Pass additional argument to update.");
                return;
            else
                disp("Calculating Workspace...");
            end
        
            % Define the range of joint angles
            q_1 = joint_limits_local(1, 1):obj.workspaceResolution:joint_limits_local(1, 2);
            q_2 = joint_limits_local(2, 1):obj.workspaceResolution:joint_limits_local(2, 2);
            q_3 = joint_limits_local(3, 1):obj.workspaceResolution:joint_limits_local(3, 2);
            q_4 = joint_limits_local(4, 1):obj.workspaceResolution:joint_limits_local(4, 2);
        
            % Initialize workspace array
            num_samples = length(q_1) * length(q_2) * length(q_3) * length(q_4);
            workspace = zeros(3, num_samples);
            n = 1;
        
            % Nested loops to iterate through all combinations of joint angles
            for i = 1:length(q_1)
                for j = 1:length(q_2)
                    for k = 1:length(q_3)
                        for l = 1:length(q_4)
                            % Set the robot's joint angles
                            obj.setQ([q_1(i); q_2(j); q_3(k); q_4(l)]);
        
                            % Calculate and store the end-effector position
                            workspace(:, n) = obj.getEndeffectorPos;
                            n = n + 1;
                        end
                    end
                end
            end
        
            % Reduce the number of workspace points using uniquetol
            uniqueWorkspace = uniquetol(workspace', obj.workspaceTolerance, 'ByRows', true)';
        
            % Compute a 3D boundary (convex hull) of the robot's reduced workspace
            K = boundary(uniqueWorkspace(1, :)', uniqueWorkspace(2, :)', uniqueWorkspace(3, :)');
            v = uniqueWorkspace'; % The vertices of the boundary
        
            % Store the boundary
            obj.boundaryK = K;
            obj.boundaryVertices = v;
        
            fprintf("Serial Workspace Calculation took %.2f seconds\n" + ...
                "BoundaryVertices Count: %d \n\n", toc, length(v));
        end

        function isInside = isPointInWorkspace(obj, point)
            % This method checks if a given point is within the robot's workspace.
        
            % Use the boundaryVertices
            vertices = obj.boundaryVertices;
            isInside = inhull(point', vertices);

        end

    end
    
    methods (Access = private)

        function closeFigureCallback(obj, src)
            % This callback function will be executed when the figure is being closed.
            % It clears the fig property of the object and then closes the figure.
            obj.fig = [];
            delete(src);
        end

        function ensureFigureExists(obj)
            if isempty(obj.fig) || ~isvalid(obj.fig)
                obj.fig = figure;

                % Set the CloseRequestFcn of the figure
                set(obj.fig, 'CloseRequestFcn', @(src, event) obj.closeFigureCallback(src));

                hold on
                view(-290, 25);
                axis equal;
                xlabel('X');
                ylabel('Y');
                zlabel('Z');
                title('Simulated Robot');
                grid on;

                % Set fixed axis limits
                xlim([-700, 700]);
                ylim([-700, 700]);
                zlim([0, 700]);
            end
        end
    end
    
    
    methods (Static, Hidden)

        %% In Hull John D'Errico
        function in = inhull(testpts,xyz,tess,tol)
            % inhull: tests if a set of points are inside a convex hull
            % usage: in = inhull(testpts,xyz)
            % usage: in = inhull(testpts,xyz,tess)
            % usage: in = inhull(testpts,xyz,tess,tol)
            %
            % arguments: (input)
            %  testpts - nxp array to test, n data points, in p dimensions
            %       If you have many points to test, it is most efficient to
            %       call this function once with the entire set.
            %
            %  xyz - mxp array of vertices of the convex hull, as used by
            %       convhulln.
            %
            %  tess - tessellation (or triangulation) generated by convhulln
            %       If tess is left empty or not supplied, then it will be
            %       generated.
            %
            %  tol - (OPTIONAL) tolerance on the tests for inclusion in the
            %       convex hull. You can think of tol as the distance a point
            %       may possibly lie outside the hull, and still be perceived
            %       as on the surface of the hull. Because of numerical slop
            %       nothing can ever be done exactly here. I might guess a
            %       semi-intelligent value of tol to be
            %
            %         tol = 1.e-13*mean(abs(xyz(:)))
            %
            %       In higher dimensions, the numerical issues of floating
            %       point arithmetic will probably suggest a larger value
            %       of tol.
            %
            %       DEFAULT: tol = 0
            %
            % arguments: (output)
            %  in  - nx1 logical vector
            %        in(i) == 1 --> the i'th point was inside the convex hull.
            %  
            % Example usage: The first point should be inside, the second out
            %
            %  xy = randn(20,2);
            %  tess = convhulln(xy);
            %  testpoints = [ 0 0; 10 10];
            %  in = inhull(testpoints,xy,tess)
            %
            % in = 
            %      1
            %      0
            %
            % A non-zero count of the number of degenerate simplexes in the hull
            % will generate a warning (in 4 or more dimensions.) This warning
            % may be disabled off with the command:
            %
            %   warning('off','inhull:degeneracy')
            %
            % See also: convhull, convhulln, delaunay, delaunayn, tsearch, tsearchn
            %
            % Author: John D'Errico
            % e-mail: woodchips@rochester.rr.com
            % Release: 3.0
            % Release date: 10/26/06
            
            % get array sizes
            % m points, p dimensions
            p = size(xyz,2);
            [n,c] = size(testpts);
            if p ~= c
              error 'testpts and xyz must have the same number of columns'
            end
            if p < 2
              error 'Points must lie in at least a 2-d space.'
            end
            
            % was the convex hull supplied?
            if (nargin<3) || isempty(tess)
              tess = convhulln(xyz);
            end
            [nt,c] = size(tess);
            if c ~= p
              error 'tess array is incompatible with a dimension p space'
            end
            
            % was tol supplied?
            if (nargin<4) || isempty(tol)
              tol = 0;
            end
            
            % build normal vectors
            switch p
              case 2
                % really simple for 2-d
                nrmls = (xyz(tess(:,1),:) - xyz(tess(:,2),:)) * [0 1;-1 0];
                
                % Any degenerate edges?
                del = sqrt(sum(nrmls.^2,2));
                degenflag = (del<(max(del)*10*eps));
                if sum(degenflag)>0
                  warning('inhull:degeneracy',[num2str(sum(degenflag)), ...
                    ' degenerate edges identified in the convex hull'])
                  
                  % we need to delete those degenerate normal vectors
                  nrmls(degenflag,:) = [];
                  nt = size(nrmls,1);
                end
              case 3
                % use vectorized cross product for 3-d
                ab = xyz(tess(:,1),:) - xyz(tess(:,2),:);
                ac = xyz(tess(:,1),:) - xyz(tess(:,3),:);
                nrmls = cross(ab,ac,2);
                degenflag = false(nt,1);
              otherwise
                % slightly more work in higher dimensions, 
                nrmls = zeros(nt,p);
                degenflag = false(nt,1);
                for i = 1:nt
                  % just in case of a degeneracy
                  % Note that bsxfun COULD be used in this line, but I have chosen to
                  % not do so to maintain compatibility. This code is still used by
                  % users of older releases.
                  %  nullsp = null(bsxfun(@minus,xyz(tess(i,2:end),:),xyz(tess(i,1),:)))';
                  nullsp = null(xyz(tess(i,2:end),:) - repmat(xyz(tess(i,1),:),p-1,1))';
                  if size(nullsp,1)>1
                    degenflag(i) = true;
                    nrmls(i,:) = NaN;
                  else
                    nrmls(i,:) = nullsp;
                  end
                end
                if sum(degenflag)>0
                  warning('inhull:degeneracy',[num2str(sum(degenflag)), ...
                    ' degenerate simplexes identified in the convex hull'])
                  
                  % we need to delete those degenerate normal vectors
                  nrmls(degenflag,:) = [];
                  nt = size(nrmls,1);
                end
            end
            
            % scale normal vectors to unit length
            nrmllen = sqrt(sum(nrmls.^2,2));
            % again, bsxfun COULD be employed here...
            %  nrmls = bsxfun(@times,nrmls,1./nrmllen);
            nrmls = nrmls.*repmat(1./nrmllen,1,p);
            
            % center point in the hull
            center = mean(xyz,1);
            
            % any point in the plane of each simplex in the convex hull
            a = xyz(tess(~degenflag,1),:);
            
            % ensure the normals are pointing inwards
            % this line too could employ bsxfun...
            %  dp = sum(bsxfun(@minus,center,a).*nrmls,2);
            dp = sum((repmat(center,nt,1) - a).*nrmls,2);
            k = dp<0;
            nrmls(k,:) = -nrmls(k,:);
            
            % We want to test if:  dot((x - a),N) >= 0
            % If so for all faces of the hull, then x is inside
            % the hull. Change this to dot(x,N) >= dot(a,N)
            aN = sum(nrmls.*a,2);
            
            % test, be careful in case there are many points
            in = false(n,1);
            
            % if n is too large, we need to worry about the
            % dot product grabbing huge chunks of memory.
            memblock = 1e6;
            blocks = max(1,floor(n/(memblock/nt)));
            aNr = repmat(aN,1,length(1:blocks:n));
            for i = 1:blocks
               j = i:blocks:n;
               if size(aNr,2) ~= length(j)
                  aNr = repmat(aN,1,length(j));
               end
               in(j) = all((nrmls*testpts(j,:)' - aNr) >= -tol,1)';
            end


        end
    end
end


    
