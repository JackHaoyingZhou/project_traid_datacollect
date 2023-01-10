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
cam_mat = [1811.1, 0.0, 813.3;              % camera intrinsics [m]
           0.0, 1815.3, 781.8;
           0.0, 0.0, 1.0];

overlay_trans = 0.6;                        % overlay transparancy

queue_size = 10;                            % probe pose averaging window
loop_freq = 5;

%% ----------------- rendering -----------------
rate = rateControl(loop_freq);
while true
%   tic
  % ***** receive marker *****
  mk_pix_msg = receive(mk_pix_sub);
  mk_rot_msg = receive(mk_rot_sub);
  mk_trans_msg = receive(mk_trans_sub);

  num_mk = mk_pix_msg.Layout.Dim.Size;
  mk_pix = reshape(mk_pix_msg.Data, 2, num_mk)';
  mk_rot = reshape(mk_rot_msg.Data, 3, 3, num_mk);
  mk_trans = reshape(mk_trans_msg.Data, 3, num_mk)';

  % ***** pose estimation *****
  probe_pos = estimateProbePose(mk_rot, mk_trans)
  if isnan(probe_pos)
    continue
  end
  
  % ***** rendering *****
  ecm_left_msg = receive(ecm_left_sub);
  ecm_left_img = readImage(ecm_left_msg);
  pa = outdas(:,:,10);

  pa_vert_reproj = reprojPAVert(ecm_left_img, pa, probe_pos, cam_mat);
  pa_reproj = reprojPA(ecm_left_img, pa, probe_pos, cam_mat);
  
  image(ecm_left_img);
  hold on;

  plot(mk_pix(:,1), mk_pix(:,2), 'r.', 'MarkerSize', 5)
  for i = 1:4
      plot(pa_vert_reproj(1,i), pa_vert_reproj(2,i), '.g', 'MarkerSize', 5);
  end

  overlay = imagesc(pa_reproj);
  overlay.AlphaData = (overlay_trans*(pa_reproj<-37)+(pa_reproj>=-37)).*(pa_reproj ~= -9999);
  colormap hot

  hold off;
  waitfor(rate);
%   toc
end

%% utilities
% *************************************************************************
% estimate probe frame using marker poses
% :param mk_rot: rotation matrix of markers
% :param mk_trans: translational vectors of markers
% :return probe_pos: estimated pa frame (i.e., probe) pose based on markers
% *************************************************************************
function [probe_pos] = estimateProbePose(mk_rot, mk_trans)
    transducer_length = 0.033;  % [m]
    transducer_width = 0.008;   % [m]
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
%     probe_pos = mk_pos(:,:,3) * T_mk3_probe;
    probe_pos = mk_pos(:,:,4) * T_mk4_probe;

    % sanity check
    if (abs(det(probe_pos) - 1.0) > 1e-4) || (sum(probe_pos(1:3,4) == -1) > 0) 
        probe_pos = nan;
    end
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

pa_vert = [0, size(pa,2)*res_x, 0, size(pa,2)*res_x;    % lateral (x)
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
pa_x = (1:size(pa,2)) * res_x;              % lateral fov, origin at top left [m]
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