clc
clear
close all

%% dataset info
imgNameFormat = './walking/rgbImage/%08.0f.ppm';
nrImages = 2234;
%% sampling period
start_time = 0;
dt = 0.1;
end_time = start_time + (nrImages-1)*dt;
time = [start_time:dt:end_time];

%% 
point_tracker_type = 'ORB'; % FAST+KLT | ORB
featureTracker = Perception.FeatureTracker(point_tracker_type);
current_img = Perception.ImageData();
prev_img = Perception.ImageData();
first_img = true;
max_match_count = -1;
matchDistThresold = 10; 
fig1 = figure(1);
ax1 = subplot(1, 1, 1);
fig2 = figure(2);
ax2 = subplot(1, 1, 1);

for iter = 1200:1500
    current_img_name = sprintf(imgNameFormat', iter-1);
    current_img.img = cv.imread(current_img_name);
    current_img.img = cv.cvtColor(current_img.img, 'RGB2GRAY');
    
    current_img = featureTracker.detectPoints(current_img);
    current_img = featureTracker.detectLines(current_img);
    
    if (first_img)
        prev_img = current_img ;
        first_img = false;
        continue;
    end
    
    % track features
    [success_pts, pt_matches, matchpts1, matchpts2] = featureTracker.trackPoints(prev_img, current_img, max_match_count);
    [success_lines, line_matches, good_matches] = featureTracker.trackLines(prev_img, current_img, matchDistThresold);
    
    if strcmp(point_tracker_type, 'FAST+KLT')
        output_img_pts = prev_img.drawOptFlowMatchedPoints(matchpts1, matchpts2);
    else
        output_img_pts = prev_img.drawORBMatchedPoints(current_img, pt_matches);
    end
    output_img_lines = prev_img.drawMatchedLines(current_img, line_matches, good_matches);
    
    cla; % Prevent stuffing too many images into the axes.
    imshow(output_img_pts, 'Parent', ax1);
    imshow(output_img_lines,  'Parent', ax2);
    drawnow;
    
    prev_img = current_img ;
    if (mod(iter, 100) == 0)
        disp(['Viewed: ',num2str(iter) ,'/',num2str(nrImages),' frames.']);
    end
    
    
%                     if obj.to_ros
%                     % send image to ros topic
%                     img_encoding = 'rgb8';
%                     RosUtils.publishImage(obj.image_publisher, output_img, img_encoding)
%                     
%                     % publish global marker poses to tf
%                     RosUtils.toRosTF(obj.tftree, obj.world_H_camera, 'world', 'camera');
%                 end
end