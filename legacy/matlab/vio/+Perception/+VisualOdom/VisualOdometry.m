classdef VisualOdometry
    properties
        cam_matrix;
        
        force_coplanar_features logical; % use homography to compute pose instead of essential matrix
        plane_normal;
        
        % the translation obtained from the essential matrix is a unit vector
        % since the depth information has been lost during projection of 3d
        % points to 2d points in the image plane. Theoretically we search for the
        % minimum norm solution of essential matrix in the nullspace. So, by decomposing E,
        % we can only get the direction of the translation. This needs to
        % be set using an external computation like a depth sensor or an
        % IMU based translation scale.
        absolute_scale double;
        
        world_H_camera (4, 4) double;
        prev_c1_H_c2 (4, 4) double                        
    end
    
    properties (Access = private, Constant)
        max_match_count = -1;
        match_ratio = 0.8;
        per_pixel_reprojerr_thresh = 1;
    end
    
    methods
        function obj = VisualOdometry(cam_matrix, to_ros, world_H_camera)                                    
            obj.cam_matrix = cam_matrix;
            rng(0,'twister'); % for random sampling
            
            obj.force_coplanar_features = false;
            obj.absolute_scale = 1.0;
            obj.plane_normal = [0 0 1]';
            
            obj.world_H_camera = world_H_camera;
            obj.prev_c1_H_c2 = eye(4);
            
            obj.to_ros = to_ros;
            
            if (obj.to_ros)
                obj.tftree = rostf;
                obj.image_publisher = rospublisher('/vo', 'sensor_msgs/Image');
            end
        end
        
        function obj = setAbsoluteScale(obj, scale)
            obj.absolute_scale = scale;
        end
        
        function obj = forceCoplanarFeatures(obj, force_coplanar_features, surface_normal)
            obj.force_coplanar_features = force_coplanar_features;
            obj.plane_normal = surface_normal;
        end
        
        function [obj, obj.w_H_camera] = process(obj, kps1, kps2)    
                if obj.force_coplanar_features
                    [obj, cam1_H_cam2, success] = obj.recoverPoseUsingHomography(kps1, kps2);
                else
                    [obj, cam1_H_cam2, success] = obj.nisterFivePoint(kps1, kps2);
                end
                if ~success
                    return
                end
                
                [Rcam, tcam] = LieGroups.SE3.extractSE3(obj.world_H_camera);
                [R12, t12] = LieGroups.SE3.extractSE3(cam1_H_cam2);
                if (obj.absolute_scale > 0.1) && (t12(3) > t12(1)) && (t12(3) > t12(2))
                    Rout = Rcam*R12;
                    tout = tcam + obj.absolute_scale*(Rcam*t12);
                    obj.world_H_camera = LieGroups.SE3.constructSE3(Rout, tout);
                    obj.prev_c1_H_c2  = cam1_H_cam2;
                else
                    return
                end                                       
        end
    end
        
    methods (Access = private)        
        function checkR = checkRotationMatrix(obj, R)
            checktrR = (abs(trace(R*R') -  3) < 1e-3);
            checkdetR = (abs(det(R) -  1) < 1e-3);
            checkdetRinv = (abs(det(R') -  1) < 1e-3);
            checkR = checkdetR && checkdetRinv && checktrR;
        end
                        
        function [obj, cam1_H_cam2, success] = nisterFivePoint(obj, kps1, kps2)
            % note that this method does not work for the points lying on
            % the same plane, use the recoverPoseUsingHomography() instead of the
            % essential matrix to recover the pose
            
            cam1_H_cam2 = eye(4);
            success = false;
            if numel(kps1) < 7 || (numel(kps1) ~= numel(kps2))
                return
            end
           
            % find essential matrix using RANSAC
            % if using normalized coordinates obtained by undistorting and unprojecting the pixels,
            % lower the threshold to much smaller values
            [E, mask] = cv.findEssentialMat(kps1, kps2, ...
                'CameraMatrix', obj.cam_matrix, 'Method', 'Ransac', 'Confidence', 0.995, 'Threshold', 3);
            
            % decompose the essential matrix to determine and
            % recover the optimal relative camera pose from the 4 possible
            % choices, usig a chirality check based on the inliers computed
            % during RANSAC for essential matrix E = [t_21]xR_21
            [R, t, ~, mask] = cv.recoverPose(E, kps1, kps2, 'Mask', mask);
            
            % if the size of output mask is zero, then none of the
            % inliers found during the RANSAC operation used to recover the poses passed the
            % chirality check , triangulated 3d points with positive depth
            % maybe trigger RANSAC again? or just skip
            checktrR = (abs(trace(R*R') -  3) < 1e-3);
            checkdetR = (abs(det(R) -  1) < 1e-3);
            checkdetRinv = (abs(det(R') -  1) < 1e-3);
            checkR = checkdetR && checkdetRinv && checktrR;
            if (isempty(mask) || ~checkR)
                cam1_H_cam2 = eye(4);
                success = false;
                return
            end
            
            % since openCV defines rotation and translation in the
            % direction the points move, not in the direction the camera
            % moves, we need to invert the rotation and translation to get
            % the camera pose.
            % TODO Verify if this is true.
            t = -R'*t;
            R = R';
            
            % better to use sim(3) than SE(3) for scale
            cam1_H_cam2 = [R t; zeros(1, 3) 1];
            success = true;
        end
        
        function [obj, cam1_H_cam2, success] = recoverPoseUsingHomography(obj, kps1, kps2)
            % we need atleast 4 matched keypoints to estimate 7 parameters
            % of the homography matrix
            cam1_H_cam2 = eye(4);
            success = false;

            [H, mask] = cv.findHomography(kps1, kps2, ...
                'Method', 'Ransac', 'Confidence', 0.990, 'RansacReprojThreshold', 3);
            if isempty(H)
                return
            end
            
            %check reprojection error,
            %if greater than 1 pixel,
            %   then its a bad homography estimate
            reproj_error = 0.0;
            for idx = 1:length(kps1)
                q = [kps1{idx} 1]';
                p =  H*[kps2{idx} 1]';
                dist = sqrt((q(1) - p(1)).^2 + ...
                    (q(2) - p(2)).^2);
                reproj_error = reproj_error+dist;
            end
            reprojerr_check = (reproj_error < length(kps1)*obj.per_pixel_reprojerr_thresh);
            
            
            if isempty(H) || (sum(mask) < 5)
                return
            end
            
            [motions, nrSolns] = cv.decomposeHomographyMat(H, eye(3));
            if nrSolns == 1
                R = motions.R{1};
                t = motions.t{1};
            else
                % do cheirality check - will result in 2 solutions
                validSolIdxArray = obj.findPoseSetWithVisibilityConstraint(motions.R, motions.n, ...
                    kps1, kps2, mask);
                if isempty(validSolIdxArray)
                    return
                elseif length(validSolIdxArray) == 1
                    idx = validSolIdxArray(1);
                    R = motions.R{idx};
                    t = motions.t{idx};
                else
                    % do surface normal check
                    [obj, R, t, success] = obj.findUniquePoseFromReducedHomographyDecomp(motions, validSolIdxArray, ...
                        kps1, mask);
                    if (~success)
                        return
                    end
                end
            end
            
            checkR = obj.checkRotationMatrix(R);
            if ~checkR
                cam1_H_cam2 = eye(4);
                success = false;
                return
            end
            
            cam1_H_cam2 = [R t; zeros(1, 3) 1];
            success = true;
        end
        
        
        function [obj, R, t, success] = findUniquePoseFromReducedHomographyDecomp(obj, motions, validSolIdxArray, points1, mask)
            if length(validSolIdxArray) ~=2
                R = eye(3);
                t = zeros(3, 1);
                success = false;
                return
            end
            idx1 = validSolIdxArray(1);
            idx2 = validSolIdxArray(2);
            
            R = motions.R{idx1};
            t = motions.t{idx1};
            
            plane_dist1 = abs(dot(obj.plane_normal, motions.n{idx1}));
            plane_dist2 = abs(dot(obj.plane_normal, motions.n{idx2}));
            
            if (abs(plane_dist1 - 1) > abs(plane_dist2 - 1))
                R = motions.R{idx2};
                t = motions.t{idx2};
            end
            
            success = true;
            
            % update the plane normal
            obj.plane_normal = R*obj.plane_normal;
            
            d = [];
            % update absolute scale
            for pidx = 1:length(points1)
                if mask(pidx)
                    p1 = obj.cam_matrix\[points1{pidx} 1]';
                    
                    z1 = obj.absolute_scale/(dot(p1, obj.plane_normal));
                    p2 = p1*z1;
                    d = [d dot(p2, obj.plane_normal)];
                end
            end
            obj.absolute_scale = mean(d);
        end
        
        function validSolIdxArray = findPoseSetWithVisibilityConstraint(obj, Rcell, ncell, points1, points2, mask)
            % find the solutions that validate the visibility constraint
            % all observed points lie in front of the camera
            validSolIdxArray = [];
            success = false;
            for idx = 1:length(Rcell)
                success = true;
                R = Rcell{idx};
                n = ncell{idx};
                for pidx = 1:length(points1)
                    if mask(pidx)
                        % normalize the image points
                        p1 = obj.cam_matrix\[points1{pidx} 1]';
                        p2 = obj.cam_matrix\[points2{pidx} 1]';
                        
                        p1normal = dot(p1, n);
                        p2normal = dot(p2, R*n);
                        % the point must be in front of the camera
                        % if not, its not a valid solution
                        if (p1normal < 0 || p2normal < 0)
                            success = false;
                            break;
                        end
                    end
                end
                if success
                    validSolIdxArray = [validSolIdxArray idx];
                end
            end
        end
        
    end
end

