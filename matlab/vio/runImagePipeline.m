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

%% options
force_coplanar_features = false;
to_ros = true;
draw_features = true;

%% cam parameters - extrinsics and intrinsics
cam_matrix = [1.3886138920000000e+03,                    0.0, 9.7393341100000000e+02; ...
    0.0, 1.3884722900000000e+03, 5.5244574000000000e+02; ...
    0.0,                    0.0,                     1];
dist_coeff =  [1e-16 1e-16 1e-16 1e-16 1e-16]';

% lap_R_imu = [0 -0.5 0.86; -1 0 0; 0 -0.86 -0.5];
% cam_R_lap = [0 1 0; 0 0 -1; -1 0 0];
% 
% w_R_imu = Core.rpy2rot(deg2rad(-126.4505), deg2rad(0), deg2rad(90.0633));
% w_R_cam = w_R_imu*(lap_R_imu')*(cam_R_lap');
% w_p_imu = [-0.048; 0.0206; 0.464];
% cam_p_imu = [0.085; -0.01; -0.112];
% w_p_cam = w_p_imu + w_R_cam*cam_p_imu;
% w_H_cam = LieGroups.SE3.constructSE3(w_R_cam, w_p_cam);

w_H_cam = eye(4);

%% get ground plane normal in cam frame
imu_g = [0.2551   -8.0637   -5.7967]';
imu_g_norm = imu_g/norm(imu_g);
imu_ground_plane_normal = -imu_g_norm;

% cam_ground_plane_normal = cam_R_lap*lap_R_imu*imu_ground_plane_normal;
cam_ground_plane_normal = [0;0; 1];

%% instantiate vo
vo = Perception.VisualOdom.VisualOdometry(cam_matrix, to_ros, w_H_cam);
vo = vo.forceCoplanarFeatures(force_coplanar_features, cam_ground_plane_normal);

%% timers and iterators

for iter = 1200:nrImages
    current_img_name = sprintf(imgNameFormat', iter-1);
    current_img = cv.imread(current_img_name);

        [vo, output_img] = vo.process(current_img, draw_features);
    
%      imshow(output_img)

%     pause(0.001);
    if (mod(iter, 100) == 0)
        disp(['Viewed: ',num2str(iter) ,'/',num2str(nrImages),' frames.']);
    end
    
end

disp(['Time taken: ', num2str(toc),' s.']);

