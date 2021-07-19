classdef ImageData  
    properties
        keypoints;
        keypoint_descriptors;
        keylines;
        keyline_descriptors;
        img;     
        binary_mask;
    end
    
    methods
        function obj = ImageData()
            obj.img = [];
            obj.keypoints = [];
            obj.keypoint_descriptors = [];
            obj.keylines = [];
            obj.keyline_descriptors = [];  
            obj.binary_mask = [];
        end
                
        function obj = updateKeyPointsUsingCornerDetector(obj, features2d)
            % instantiate a fast corner detector using
            % fast_detector = cv.FastFeatureDetector() and pass the object to this function
            % or instantiate an ORB tracker using
            % orb_tracker = cv.ORB();
            % orb_tracker.MaxFeatures = 1000;
            % and pass the object to this function
            assert(~isempty(obj.img), 'No image found');
            [obj.keypoints, obj.keypoint_descriptors] = features2d.detectAndCompute(obj.img);
        end
        
        function obj = updateKeyLinesUsingLSDetectorAndBinaryDescriptor(obj, lsd, binaryDesc)
            % instantiate a fast corner detector using
            % cv.BinaryDescriptorMatcher() and pass the object to this function
            assert(~isempty(obj.img), 'No image found');
            obj.binary_mask = ones(size(obj.img, 1), size(obj.img, 2), 'uint8');
            obj.keylines = lsd.detect(obj.img, 'Scale', 2, 'NumOctaves', 2, 'Mask', obj.binary_mask); 
            obj.keyline_descriptors = binaryDesc.compute(obj.img, obj.keylines);
            % select lines and desciptors from second octave
            idx = ([obj.keylines.octave] == 1);
            obj.keylines = obj.keylines(idx);
            obj.keyline_descriptors = obj.keyline_descriptors(idx, :);
        end
                
        function [success, matches, matchpts1, matchpts2] = trackKeyPointsUsingFeatureDesc(obj, other, desc_matcher, max_match_count)
            % this image will be the query image (previous image frame)
            % other image will be the trained image (current image frame)
            success = false;
            matchpts1 = [];
            matchpts2 = [];            
            
            if (numel(obj.keypoints) < 2)  
                disp('Returning this image without tracking point features due to non-availability of enough keypoints');
                return;
            end
            
            % two nearest neighbour matching followed by
            % LOWE's ratio test to obtain good matches
            % to get distinct far-away keypoint neighbours
            bestMatchCount = 2; % Count of best matches found per each query descriptor
            matches = desc_matcher.knnMatch(obj.keypoint_descriptors, ...
                                            other.keypoint_descriptors, bestMatchCount);
            match_ratio = 0.8;
            idx = cellfun(@(m) numel(m) == 2 && ...
                         (m(1).distance < match_ratio*m(2).distance), matches);
            matches = cellfun(@(m) m(1), matches(idx));
            
            if isempty(matches) || (numel(matches) < 4)
                return;
            end
            
            if (max_match_count < 0)
                % choose all matches
            else
                [~, ord] = sort([matches.distance]);
                ord(max_match_count+1:end) = [];
                matches = matches(ord);
            end
                        
            % get matched points
            points1 = cat(1, obj.keypoints([matches.queryIdx]+1));
            points2 = cat(1, other.keypoints([matches.trainIdx]+1));
                        
            point_count = length(points1);
            matchpts1 = cell(1, point_count);
            matchpts2 = cell(1, point_count);
            for idx=1:point_count
                matchpts1{idx} = points1(idx).pt;
                matchpts2{idx} = points2(idx).pt;
            end            
            success = true;            
        end        
        
        function [success, matches, matchpts1, matchpts2] = trackFeaturesUsingOpticalFlow(obj, other)  
            % this image will be the query image (previous image frame)
            % other image will be the trained image (current image frame)
            klt_params = {'WinSize',[21 21], 'MaxLevel',3, ...
                          'Criteria', struct('type','Count+EPS', 'maxCount',30, 'epsilon',0.01)};
            point_count = length(obj.keypoints);
            p1 = cell(1, point_count);
            for idx = 1:point_count
                p1{idx} = obj.keypoints(idx).pt;
            end
            
            % calculate sparse optical flow in both directions
            p2 = cv.calcOpticalFlowPyrLK(obj.img, other.img, p1, ...
                                         klt_params{:});
            p1r = cv.calcOpticalFlowPyrLK(other.img, obj.img, p2, klt_params{:});
            matches = cellfun(@(a, b) max(abs(a-b)), p1, p1r) < 1.0;
            
            matchpts1 = p1(matches);
            matchpts2 = p2(matches);
                                    
            success = true;
        end
                                                        
        function [success, line_matches, good_matches] = trackLinesUsingBinaryDescMatcher(obj, other, descMatcher, matchDistThresold)
            success = false;
            line_matches = [];            
            
            if (numel(obj.keylines) < 2)  
                disp('Returning this image without tracking lines features due to non-availability of enough keylines');                
                return;
            end
            
            line_matches =  descMatcher.match(obj.keyline_descriptors, ...
                                         other.keyline_descriptors);
            good_matches = ([line_matches.distance] < matchDistThresold);            
            success = true;                     
        end
        
        function output_img = drawOptFlowMatchedPoints(obj, matchpts1, matchpts2)
             output_img = cv.cvtColor(obj.img, 'GRAY2RGB');
             output_img = cv.line(output_img, matchpts1, matchpts2, 'Color', [0 128 0]);
             output_img = cv.circle(output_img, matchpts1, 2, 'Thickness', 'Filled', 'Color', [255 0 0]);
        end
        
        function output_img = drawORBMatchedPoints(obj, other, pt_matches)
            output_img = [];
            assert(~isempty(obj.img), 'This image is empty');
            assert(~isempty(other.img), 'Other image is empty.');
                          
            if (isempty(obj.keypoints) || isempty(other.keypoints) || isempty(pt_matches))
                disp('No matched keypoints or nothing to draw.')
                return
            end
            output_img = cv.drawMatches(obj.img, ...
                obj.keypoints, ...
                other.img, ...
                other.keypoints, pt_matches);            
        end
        
        function output_img = drawMatchedLines(obj, other, line_matches, good_matches)
            output_img = [];
            assert(~isempty(obj.img), 'This image is empty');
            assert(~isempty(other.img), 'Other image is empty.');
            
            if (isempty(obj.keylines) || isempty(other.keylines) || isempty(line_matches))
                disp('No matched keylines or nothing to draw.')
                return
            end
            
            img1 = cv.resize(obj.img, 0.5, 0.5);
            img2 = cv.resize(other.img, 0.5, 0.5);
            
            output_img = cv.drawLineMatches(img1,  ...
                                            obj.keylines, ...
                                            img2, ...
                                            other.keylines, ...
                                            line_matches, 'MatchesMask', good_matches, 'MatchColor', [0 255 0]);               
        end
    end
end

