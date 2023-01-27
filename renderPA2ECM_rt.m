% =============================================================================
% file name:    renderPA2ECM_rt.m
% description:  render PA image to ecm view online based on tracked marker
% author:       Xihan Ma
% date:         2022-12-30
% =============================================================================
clc; clear; close all;
addpath('utils/');

%% ----------------- ROS network -----------------
rosshutdown
% ***** ROS_IP and ROS_MASTER_URI should already be correctly configured *****
rosinit
mk_pix_sub = rossubscriber('/marker/pix', 'std_msgs/Int32MultiArray');
mk_rot_sub = rossubscriber('/marker/rot', 'std_msgs/Float32MultiArray');
mk_trans_sub = rossubscriber('/marker/trans', 'std_msgs/Float32MultiArray');

ecm_left_sub = rossubscriber('/cv_camera_left_0/image_raw', 'sensor_msgs/Image');
% pa_sub = rossubscriber('/PA_IMG', 'sensor_msgs/Image', @pa_sub_cb);
% pa_sub = rossubscriber('/PA_IMG', 'std_msgs/Float64MultiArray', @pa_sub_cb);
record_ready_flag_sub = rossubscriber('/PA_ready_flag', 'std_msgs/Bool', @record_ready_flag_cb);

%% ----------------- rendering -----------------
% ********** const params **********
cam_mat = [596.5596, 0.0, 261.1265;         % camera intrinsics [m]
           0.0, 790.1609, 238.1423;
           0.0, 0.0, 1.0];
ECM_DISP_WIDTH = 640;                       % ecm display width 
ECM_DISP_HEIGHT = 480;                      % ecm display height
overlay_trans = 0.6;                        % overlay transparancy
pose_queue_size = 10;                       % probe pose averaging window
pose_queue_count = 1;                       % probe pose averaging window count
num_mk = 4;                                 % mk_pix_msg.Layout.Dim.Size;
loop_freq = 20;                             % max rendering loop frequency
rate = rateControl(loop_freq);              % ROS loop rate control
isSaveRender = false;                       % if save ecm w/ overlay during data collection

mk_trans_que = -1*ones(num_mk, 3, pose_queue_size);
mk_rot_que = -1*ones(3, 3, num_mk, pose_queue_size);

% ********** global var parsing between callbacks **********
global probe_pos            % current probe pose
global probe_pos_rec        % probe pose recording
global mk_trans_avg         % current marker position
global mk_trans_avg_rec     % marker position recording
global rec_count            % probe pose recording count
global rec_size             % total pose recording
% global pa_volume            % beamformed pa images

pa = rand(128, 800);    % for testing only

rec_size = 41;  % number of data samples to record
rec_count = 1;
probe_pos_rec = zeros(4, 4, rec_size);
probe_pos = eye(4);
% pa_volume = zeros(128, 800, rec_size);

mk_rot_avg = -1*ones(3,3,num_mk);
mk_trans_avg = -1*ones(num_mk, 3);
mk_trans_avg_rec = -1*ones(num_mk, 3, rec_size);

% ********** main loop **********
close all; f = figure(WindowKeyPressFcn={@figureCallback, isSaveRender});
while true
    %   tic
    % ***** receive marker pose *****
    mk_pix_msg = receive(mk_pix_sub);
    mk_rot_msg = receive(mk_rot_sub);
    mk_trans_msg = receive(mk_trans_sub);

    mk_pix = reshape(mk_pix_msg.Data, 2, num_mk)';
    mk_rot = reshape(mk_rot_msg.Data, 3, 3, num_mk);
    mk_trans_raw = reshape(mk_trans_msg.Data, 3, num_mk)';

    mk_trans = filterMarkerDepth(mk_trans_raw);
    % TODO: filter z axis rotation
    
    % queue-up
    mk_trans_que(:, :, pose_queue_count) = mk_trans;
    mk_rot_que(:, :, :, pose_queue_count) = mk_rot;
    if pose_queue_count < pose_queue_size
        pose_queue_count = pose_queue_count + 1;
    else
        pose_queue_count = 1;
    end

    mk_trans_que(mk_trans_que == -1) = nan;
    mk_trans_avg(:,1) = mean(squeeze(mk_trans_que(:,1,:)), 2, 'omitnan');
    mk_trans_avg(:,2) = mean(squeeze(mk_trans_que(:,2,:)), 2, 'omitnan');
    mk_trans_avg(:,3) = mean(squeeze(mk_trans_que(:,3,:)), 2, 'omitnan');

    mk_rot_que(mk_rot_que == -1) = nan;
    mk_rot_x = mean(reshape(mk_rot_que(:,1,:,:), 3, 4, pose_queue_size), 3, 'omitnan');   % per-marker avg over frames
    mk_rot_y = mean(reshape(mk_rot_que(:,2,:,:), 3, 4, pose_queue_size), 3, 'omitnan');   % per-marker avg over frames
    mk_rot_z = mean(reshape(mk_rot_que(:,3,:,:), 3, 4, pose_queue_size), 3, 'omitnan');   % per-marker avg over frames
    mk_rot_avg(:,1,:) = mk_rot_x;
    mk_rot_avg(:,2,:) = mk_rot_y;
    mk_rot_avg(:,3,:) = mk_rot_z;

    % ***** pose estimation *****
    [probe_pos, ~] = estimatePoseMultipleMarkers(mk_rot_avg, mk_trans_avg);
%     estimateProbeDim(mk_trans_avg);

    if isnan(probe_pos)
        disp('probe pose estimation failed')
        continue
    end

    % ***** rendering *****
    ecm_left_msg = receive(ecm_left_sub);
    ecm_left_img = readImage(ecm_left_msg);
    ecm_left_img = imresize(ecm_left_img, [ECM_DISP_HEIGHT, ECM_DISP_WIDTH]);
    image(ecm_left_img); axis image off;
    hold on;

    pa_vert_reproj = reprojPAVert(ecm_left_img, pa, probe_pos, cam_mat);
    plot(mk_pix(:,1), mk_pix(:,2), 'r.', 'MarkerSize', 10)
    plot(pa_vert_reproj(1,:), pa_vert_reproj(2,:), 'xg', 'MarkerSize', 5);    

%     pa_reproj = reprojPA(ecm_left_img, pa, probe_pos, cam_mat);
%     overlay = imagesc(pa_reproj);
%     overlay.AlphaData = (overlay_trans*(pa_reproj<-37)+(pa_reproj>=-37)).*(pa_reproj ~= -9999);
%     colormap hot

    hold off;
    waitfor(rate);
    %   toc
end

%% ----------------- save data -----------------
save_path = 'probe_pose_recording/';
time = datestr(now, 'dd-mmm-yyyy-hh-MM');
tag = '_pa720';
save([save_path, time, tag, '_mk_trans.mat'],'mk_trans_avg_rec');
save([save_path, time, tag, '_probe_pose.mat'],'probe_pos_rec');

%% check marker surface normal estimation
figure();
plot3(mk_trans_avg(:,1),mk_trans_avg(:,2),mk_trans_avg(:,3),'.g','MarkerSize',10);
grid on; hold on; axis equal;

% orig = mean(mk_trans_avg, 'omitnan');
% norm = estimateNormal(mk_trans_avg);

orig = probe_pos(1:3,4);
norm = probe_pos(1:3,3);

scale = 0.02;
plot3(orig(1)+scale*norm(1),orig(2)+scale*norm(2),orig(3)+scale*norm(3),'.k','MarkerSize',10)
plot3(orig(1),orig(2),orig(3),'.r','MarkerSize',10)
legend('markers','norm','orig')

%% debug section
estimatePoseMultipleMarkers(mk_rot_avg, mk_trans_avg)

%% utilities
% *************************************************************************
% save information in current frame
% *************************************************************************
function saveCurrFrame()

global probe_pos            % current probe pose
global probe_pos_rec        % probe pose recording
global mk_trans_avg         % current marker position
global mk_trans_avg_rec     % marker position recording
global rec_count            % probe pose recording count
global rec_size             % total pose recording
% global pa_volume            % beamformed pa image

probe_pos_rec(:,:,rec_count) = probe_pos;
mk_trans_avg_rec(:,:,rec_count) = mk_trans_avg;
% imwrite(pa_volume(:,:,rec_count), ['pa_frm', num2str(rec_count), '.png']);

if rec_count < rec_size
    rec_count = rec_count + 1;
else
    rec_count = 1;
end

fprintf('probe pose recorded, count: %d\n', rec_count-1)
disp(probe_pos)

end

% *************************************************************************
% overlay window callback
% *************************************************************************
function figureCallback(src, event, isSaveRender)

    if event.Character == "s"
        saveCurrFrame();
    end

    if isSaveRender
        saveas(src, ['ecm_snapshot', num2str(rec_count), '.jpg'])
    end

end

% *************************************************************************
% ROS topic subscriber callback: /record_ready_flag
% *************************************************************************
function record_ready_flag_cb(~, message)

if message.Data == true
    disp('record flag received')
    saveCurrFrame();
end

end

% *************************************************************************
% ROS topic subscriber callback: /pa_image
% *************************************************************************
function pa_sub_cb(~, message)

global pa_volume   % beamformed pa image
global rec_count   % probe pose recording count

% pa = readImage(message);
pa = reshape(message.Data, 800, 128);
pa_volume(:, :, rec_count) = pa;

end

% *************************************************************************
% reject unreliable marker z position
% *************************************************************************
function [mk_trans_filt] = filterMarkerDepth(mk_trans)

mk_trans_filt = mk_trans;
z_diff = zeros(1,size(mk_trans,1)); 
% calculate difference between each z and all other z
for i = 1:size(mk_trans,1)
    z_diff(i) = sum(abs(mk_trans(i,3) - mk_trans(setdiff(1:size(mk_trans,1), i),3)));
end
[max_diff, max_ind] = max(z_diff);
if max_diff > 0.01
    mk_trans_filt(max_ind, :) = -1;
end

end
