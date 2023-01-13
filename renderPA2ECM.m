% =============================================================================
% file name:    renderPA2ECM.m
% description:  render PA image to ecm view based on tracked marker
% author:       Xihan Ma
% date:         2022-12-30
% =============================================================================
clc; clear; close all;

%% ----------------- ROS network -----------------
rosshutdown
% ***** ROS_IP and ROS_MASTER_URI should already be correctly configured *****
rosinit

% TODO: use callback to receive pose & average in a time window
ecm_left_sub = rossubscriber('/cv_camera_left_0/image_raw', 'sensor_msgs/Image');
mk_pix_sub = rossubscriber('/marker/pix', 'std_msgs/Int32MultiArray');
mk_rot_sub = rossubscriber('/marker/rot', 'std_msgs/Float32MultiArray');
mk_trans_sub = rossubscriber('/marker/trans', 'std_msgs/Float32MultiArray');

%% ----------------- params -----------------
cam_mat = [596.5596, 0.0, 261.1265;           % camera intrinsics [m]
    0.0, 790.1609, 238.1423;
    0.0, 0.0, 1.0];

overlay_trans = 0.6;                        % overlay transparancy

pose_queue_size = 10;                            % probe pose averaging window
pose_queue_count = 1;

num_mk = 4; % mk_pix_msg.Layout.Dim.Size;
loop_freq = 20;

rec_size = 50;  % number of data samples to record
rec_count = 1;

%% ----------------- rendering -----------------
rate = rateControl(loop_freq);
pa = outdas(:,:,10);    % for testing only
mk_trans_que = -1*ones(num_mk, 3, pose_queue_size);
mk_rot_que = -1*ones(3, 3, num_mk, pose_queue_size);

isRecData = false;
mk_trans_avg_rec = -1*ones(num_mk, 3, rec_size);

mk_trans_avg = -1*ones(num_mk, 3);
mk_rot_avg = -1*ones(3,3,num_mk);
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

    % *************** for data recording ***************
    if isRecData
        mk_trans_avg_rec(:,:,rec_count) = mk_trans_avg;
        if rec_count < rec_size
            rec_count = rec_count + 1;
        else
            rec_count = 1;
        end
    end
    % **************************************************

    % ***** pose estimation *****
%     [probe_pos, mk_trans_recalc] = estimatePoseMultipleMarkers(mk_rot_avg, mk_trans_avg)
    [probe_pos, ~] = estimatePoseMultipleMarkers(mk_rot_avg, mk_trans_avg)
%     estimateProbeDim(mk_trans_avg);

    if isnan(probe_pos)
        continue
    end

    % ***** rendering *****
    ecm_left_msg = receive(ecm_left_sub);
    ecm_left_img = readImage(ecm_left_msg);
    ecm_left_img = imresize(ecm_left_img, [480, 640]);
    image(ecm_left_img); axis image off;
    hold on;

%     pa_vert_reproj = reprojPAVert(ecm_left_img, pa, probe_pos, cam_mat);
%     plot(mk_pix(:,1), mk_pix(:,2), 'r.', 'MarkerSize', 5)
%     for i = 1:4
%         plot(pa_vert_reproj(1,i), pa_vert_reproj(2,i), '.g', 'MarkerSize', 5);
%     end

    pa_reproj = reprojPA(ecm_left_img, pa, probe_pos, cam_mat);
    overlay = imagesc(pa_reproj);
    overlay.AlphaData = (overlay_trans*(pa_reproj<-37)+(pa_reproj>=-37)).*(pa_reproj ~= -9999);
    colormap hot

    hold off;
    waitfor(rate);
    %   toc
end

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
% estimate normal vector
% *************************************************************************
function norm = estimateNormal(points, method)

if nargin < 2
    method = 2;
end

if method == 1
    vecs = points - orig;
    norms = zeros(size(vecs,1),3);
    for i = 1:size(vecs, 1)
        if i == length(points)
            norms(i,:) = -normalize(cross(vecs(i,:), vecs(1,:)), 'norm');
        else
            norms(i,:) = -normalize(cross(vecs(i,:), vecs(i+1,:)), 'norm');
        end
        if norms(i, 3) < 0
            norms(i, :) = - norms(i, :);
        end
    end

elseif method == 2
    norms = zeros(size(points,1), 3);
    P1_ind = [2, 3, 4, 1];
    P2_ind = [3, 4, 1, 2];
    for i = 1:size(norms,1)
        P0 = points(i,:);
        P1 = points(P1_ind(i), :);
        P2 = points(P2_ind(i), :);
        norms(i, :) = normalize(cross(P1-P0, P2-P0), 'norm');
        if norms(i, 3) < 0
            norms(i, :) = - norms(i, :);
        end
    end
end

norm = mean(norms, 'omitnan');

end

% *************************************************************************
% reject unreliable marker z position
% *************************************************************************
function [mk_trans_filt] = filterMarkerDepth(mk_trans)

mk_trans_filt = mk_trans;
z_diff = zeros(1,size(mk_trans,1)); % difference between each z and all other z
for i = 1:size(mk_trans,1)
    z_diff(i) = sum(abs(mk_trans(i,3) - mk_trans(setdiff(1:size(mk_trans,1), i),3)));
end
[max_diff, max_ind] = max(z_diff);
if max_diff > 0.01
    mk_trans_filt(max_ind, :) = -1;
end

end

% *************************************************************************
% estimate probe length & probe width
% *************************************************************************
function [probe_length, probe_width] = estimateProbeDim(mk_trans)

    length1 = norm(mk_trans(1,:) - mk_trans(2,:));
    width1 = norm(mk_trans(1,:) - mk_trans(3,:));
    length2 = norm(mk_trans(3,:) - mk_trans(4,:));
    width2 = norm(mk_trans(2,:) - mk_trans(4,:));
    fprintf('probe length: %f, %f, probe width: %f, %f\n', ...
        length1, length2, width1, width2)

    probe_length = mean([length1, length2], 'omitnan');
    probe_width = mean([width1, width2], 'omitnan');
    
end

% *************************************************************************
% complete marker positions (x,y,z) based on known marker spatial relations
% *************************************************************************
function [mk_trans_complete] = completeMarkerPos(mk_rot, mk_trans)

rotx = @(t) [1 0 0; 0 cosd(t) -sind(t) ; 0 sind(t) cosd(t)];
% roty = @(t) [cosd(t) 0 sind(t) ; 0 1 0 ; -sind(t) 0  cosd(t)];
% rotz = @(t) [cosd(t) -sind(t) 0 ; sind(t) cosd(t) 0 ; 0 0 1];

transducer_length = 0.033;  % [m]
transducer_width = 0.008;   % [m]

mk_ids = 1:size(mk_trans, 1);
mk_detected = mk_ids(sum(isnan(mk_trans),2) == 0);

mk_trans_complete = mk_trans;

for i = 1:length(mk_detected)
    mk_id = mk_detected(i);
    switch mk_id
        case 1
            % calc mk2 position
            mk2 = mk_trans_complete(mk_id,:) - mk_rot(:,1,mk_id)' * transducer_length;
            mk_trans_complete(2,:) = mean([mk2; mk_trans_complete(2,:)], 'omitnan');
            % calc mk3 position
            R = mk_rot(:,:,mk_id) * rotx(-120); % -30
            mk3 = mk_trans_complete(mk_id,:) + R(:,2)' * transducer_width;
            mk_trans_complete(3,:) = mean([mk3; mk_trans_complete(3,:)], 'omitnan');
            % calc mk4 position
            mk4 = mk3 - mk_rot(:,1,mk_id)' * transducer_length;
            mk_trans_complete(4,:) = mean([mk4; mk_trans_complete(4,:)], 'omitnan');

        case 2
            % calc mk1 position
            mk1 = mk_trans_complete(mk_id,:) + mk_rot(:,1,mk_id)' * transducer_length;
            mk_trans_complete(1,:) = mean([mk1; mk_trans_complete(1,:)], 'omitnan');
            % calc mk4 position
            R = mk_rot(:,:,mk_id) * rotx(-120); % -30
            mk4 = mk_trans_complete(mk_id,:) + R(:,2)' * transducer_width;
            mk_trans_complete(4,:) = mean([mk4; mk_trans_complete(4,:)], 'omitnan');
            % calc mk3 position
            mk3 = mk4 + mk_rot(:,1,mk_id)' * transducer_length;
            mk_trans_complete(3,:) = mean([mk3; mk_trans_complete(3,:)], 'omitnan');

        case 3
            % calc mk4 position
            mk4 = mk_trans_complete(mk_id,:) - mk_rot(:,1,mk_id)' * transducer_length;
            mk_trans_complete(4,:) = mean([mk4; mk_trans_complete(4,:)],'omitnan');
            % calc mk2 position
            R = mk_rot(:,:,mk_id) * rotx(-120);
            mk2 = mk4 - R(:,2)' * transducer_width;
            mk_trans_complete(2,:) = mean([mk2; mk_trans_complete(2,:)], 'omitnan');
            % calc mk1 position
            mk1 = mk2 + mk_rot(:,1,mk_id)' * transducer_length;
            mk_trans_complete(1,:) = mean([mk1; mk_trans_complete(1,:)], 'omitnan');

        case 4
            % calc mk3 position
            mk3 = mk_trans_complete(mk_id,:) + mk_rot(:,1,mk_id)' * transducer_length;
            mk_trans_complete(3,:) = mean([mk3; mk_trans_complete(3,:)], 'omitnan');
            % calc mk2 position
            R = mk_rot(:,:,mk_id) * rotx(-120);
            mk2 = mk_trans_complete(mk_id,:) - R(:,2)' * transducer_width;
            mk_trans_complete(2,:) = mean([mk2; mk_trans_complete(2,:)], 'omitnan');
            % calc mk1 position
            mk1 = mk2 + mk_rot(:,1,mk_id)' * transducer_length;
            mk_trans_complete(1,:) = mean([mk1; mk_trans_complete(1,:)], 'omitnan');
    
    end
end

end

% *************************************************************************
% estimate probe frame using single marker pose (deprecated)
% :param mk_rot: rotation matrix of markers
% :param mk_trans: translational vectors of markers
% :return probe_pos: estimated pa frame (i.e., probe) pose based on markers
% *************************************************************************
function [probe_pos] = estimatePoseSingleMarker(mk_rot, mk_trans)

transducer_length = 0.033;  % [m]
transducer_width = 0.008;   % [m]
% TODO: update rotation part
T_mk1_probe = [1, 0, 0, -transducer_length/2;
    0, -1, 0, transducer_width/2;
    0, 0, -1, 0;
    0, 0, 0, 1];

T_mk2_probe = [1, 0, 0, transducer_length/2;
    0, -1, 0, transducer_width/2;
    0, 0, -1, 0;
    0, 0, 0, 1];

T_mk3_probe = [-1, 0, 0, transducer_length/2;
    0, 1, 0, transducer_width/2;
    0, 0, -1, 0;
    0, 0, 0, 1];

T_mk4_probe = [-1, 0, 0, -transducer_length/2;
    0, 1, 0, transducer_width/2;
    0, 0, -1, 0;
    0, 0, 0, 1];

num_mk = size(mk_rot,3);
mk_pos = zeros(4,4,num_mk);
for i = 1:num_mk
    mk_pos(:,:,i) = eye(4);
    mk_pos(1:3,1:3,i) = mk_rot(:,:,i);
    if mk_pos(3,3,i) > 0
        mk_pos(3,3,i) = - mk_pos(3,3,i);    % force z-axis pointing to same direction
    end
    mk_pos(1:3,4,i) = mk_trans(i,:);
end

%     probe_pos = mk_pos(:,:,1) * T_mk1_probe;
%     probe_pos = mk_pos(:,:,2) * T_mk2_probe;
probe_pos = mk_pos(:,:,3) * T_mk3_probe;
%     probe_pos = mk_pos(:,:,4) * T_mk4_probe;

% sanity check
if (abs(det(probe_pos) - 1.0) > 1e-4) || (sum(probe_pos(1:3,4) == -1) > 0)
    probe_pos = nan;
end

end

% *************************************************************************
% estimate probe pose using marker poses
% *************************************************************************
function [probe_pos, mk_trans] = estimatePoseMultipleMarkers(mk_rot, mk_trans)

transducer_length = 0.033;  % [m]
transducer_width = 0.008;   % [m]
transducer_depth = 0.007;   % [m]

num_mk_detected = sum(sum(isnan(mk_trans),2) == 0);
fprintf('detected markers: %d', num_mk_detected);
if num_mk_detected < 3
    mk_trans = completeMarkerPos(mk_rot, mk_trans);
end

probe_pos = eye(4);

x_vecs = [mk_trans(1,:) - mk_trans(2,:); ...
    mk_trans(3,:) - mk_trans(4,:)];
x_vec = normalize(mean(x_vecs, 'omitnan'), 'norm');

y_vecs = [mk_trans(1,:) - mk_trans(3,:); ...
    mk_trans(2,:) - mk_trans(4,:)];
y_vec = normalize(mean(y_vecs, 'omitnan'), 'norm');

% z_vec = estimateNormal(mk_trans);
z_vec = cross(x_vec, y_vec);

%     orig = mean(mk_trans,'omitnan');  % this only works when all markers are visible

orig = zeros(3, 1);
x1 = mk_trans(1,1) - x_vec*transducer_length/2;
x2 = mk_trans(2,1) + x_vec*transducer_length/2;
x3 = mk_trans(3,1) - x_vec*transducer_length/2;
x4 = mk_trans(4,1) + x_vec*transducer_length/2;
x12 = mean([x1, x2]);
x34 = mean([x3, x4]);
orig(1) = mean([x12, x34], 'omitnan');

y1 = mk_trans(1,2) + y_vec*transducer_width/2;
y2 = mk_trans(2,2) - y_vec*transducer_width/2;
y3 = mk_trans(3,2) + y_vec*transducer_width/2;
y4 = mk_trans(4,2) - y_vec*transducer_width/2;
y13 = mean([y1, y3]);
y24 = mean([y2, y4]);
orig(2) = mean([y13, y24], 'omitnan');

z1 = mk_trans(1,3) + z_vec*transducer_depth;
z2 = mk_trans(2,3) + z_vec*transducer_depth;
z3 = mk_trans(3,3) + z_vec*transducer_depth;
z4 = mk_trans(4,3) + z_vec*transducer_depth;
z14 = mean([z1, z4]);
z23 = mean([z2, z3]);
orig(3) = mean([z14, z23],'omitnan');

probe_pos(1:3, 1) = x_vec;
probe_pos(1:3, 2) = y_vec;
probe_pos(1:3, 3) = z_vec;
probe_pos(1:3, 4) = orig;

end

% *************************************************************************
% reproject four verticies of PA image to ECM image frame
% :param ecm: ecm image
% :param pa: pa image
% :param T_ecm_pa: transformation from ecm frame to pa frame
% :param cam_intri: camera intrinsic matrix
% :return pa_vert_reproj: verticies of pa image under ecm frame
% *************************************************************************
function [pa_vert_reproj] = reprojPAVert(ecm, pa, T_ecm_pa, cam_intri)

res_x = 0.253*1e-3;                 % lateral resolution [m/pix]
res_z = 0.053*1e-3;                         % axial resolution [m/pix]

pa_vert = [-size(pa,2)/2*res_x, size(pa,2)/2*res_x, -size(pa,2)/2*res_x, size(pa,2)/2*res_x;    % lateral (x)
    0, 0, size(pa,1)*res_z, size(pa,1)*res_z];   % axial (z)
pa_vert_reproj = zeros(size(pa_vert));

for v = 1:4
    pix_pa_xyz = [pa_vert(1,v); 0; pa_vert(2,v); 1];
    pix_ecm_xyz = T_ecm_pa * pix_pa_xyz;
    pix_ecm_uv = cam_intri * pix_ecm_xyz(1:3);
    pix_ecm_uv = pix_ecm_uv./pix_ecm_uv(end);

    pix_ecm_uv(pix_ecm_uv <= 0) = 1;
    pix_ecm_v = min(ceil(pix_ecm_uv(1)), size(ecm,2));
    pix_ecm_u = min(ceil(pix_ecm_uv(2)), size(ecm,1));
    pa_vert_reproj(:, v) = [pix_ecm_v, pix_ecm_u];
end

end

% *************************************************************************
% reproject each pixel PA image to ECM image frame
% :param ecm: ecm image
% :param pa: pa image
% :param T_ecm_pa: transformation from ecm frame to pa frame
% :param cam_intri: camera intrinsic matrix
% :return pa_reproj: per-pixel reprojection of pa image under ecm frame
% *************************************************************************
function [pa_reproj] = reprojPA(ecm, pa, T_ecm_pa, cam_intri)

res_x = 0.253*1e-3;                 % lateral resolution [m/pix]
res_z = 0.053*1e-3;                           % axial resolution [m/pix]
pa_x = ((1:size(pa,2)) - size(pa,2)/2) * res_x;              % lateral fov, origin at top left [m]
pa_z = (1:size(pa,1)) * res_z;              % axial fov, origin at top left [m]

pa_reproj = nan(size(ecm,1:2));
for row = 1:size(pa, 1)
    for col = 1:size(pa,2)
        pix_pa_xyz = [pa_x(col); 0; pa_z(row); 1];
        pix_ecm_xyz = T_ecm_pa * pix_pa_xyz;
        pix_ecm_uv = cam_intri * pix_ecm_xyz(1:3);
        pix_ecm_uv = pix_ecm_uv./pix_ecm_uv(end);

        pix_ecm_uv(pix_ecm_uv <= 0) = 1;
        pix_ecm_v = min(ceil(pix_ecm_uv(1)), size(ecm,2));
        pix_ecm_u = min(ceil(pix_ecm_uv(2)), size(ecm,1));
        pa_reproj(pix_ecm_u, pix_ecm_v) = pa(row, col);
    end
end

end