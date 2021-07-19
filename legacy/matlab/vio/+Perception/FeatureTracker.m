classdef FeatureTracker   
    properties
        orb_tracker;
        orb_matcher;       
        
        point_tracker_type; %'ORB'|'FAST+KLT'
        
        linesegment_detector;
        line_binary_desc;
        line_binary_desc_matcher;                
    end
    
    methods
        function obj = FeatureTracker(pointTrackerType)
            if nargin ~= 1
                obj.point_tracker_type = 'FAST+KLT';
            else
                obj.point_tracker_type = pointTrackerType;     
            end            
            [obj.orb_tracker, obj.orb_matcher] = Perception.instantiateORBDetectorAndMatcher();
                        
            obj.linesegment_detector = cv.LSDDetector();
            obj.line_binary_desc = cv.BinaryDescriptor();
            obj.line_binary_desc_matcher = cv.BinaryDescriptorMatcher();    
            disp(['Instantiated Feature tracker with ', obj.point_tracker_type, ' for tracking point features and LSD+LBD for tracking lines.']);
        end
        
        function imgData = detectPoints(obj, imgData)
                imgData = imgData.updateKeyPointsUsingCornerDetector(obj.orb_tracker);
        end
        
        function imgData = detectLines(obj, imgData)
            imgData = imgData.updateKeyLinesUsingLSDetectorAndBinaryDescriptor(obj.linesegment_detector, ...
                obj.line_binary_desc);
        end
        
        function [success, matches, matchpts1, matchpts2] = trackPoints(obj, prevImg, currentImg, max_match_count)
            if strcmp(obj.point_tracker_type, 'FAST+KLT')
                [success, matches, matchpts1, matchpts2] = ...
                 prevImg.trackFeaturesUsingOpticalFlow(currentImg);
            elseif strcmp(obj.point_tracker_type, 'ORB')
                [success, matches, matchpts1, matchpts2] = ...
                 prevImg.trackKeyPointsUsingFeatureDesc(currentImg, obj.orb_matcher, max_match_count);
            end
        end
        
        function [success, line_matches, good_matches] = trackLines(obj, prevImg, currentImg, matchDistThresold)
            [success, line_matches, good_matches] = prevImg.trackLinesUsingBinaryDescMatcher(currentImg, ...
                obj.line_binary_desc_matcher, matchDistThresold);
        end
        
    end
end

