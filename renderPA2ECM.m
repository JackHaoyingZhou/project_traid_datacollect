% =============================================================================
% file name:    renderPA2ECM.m
% description:
% author:       Xihan Ma
% date:         2022-12-30
% =============================================================================
clc; clear; close all;
addpath('utils/');

%% load data
expRoundId = 2;
dataFolder = '1-26-2023/';
probe_pos_data = {
                    'T1_pa700/26-Jan-2023-17-27_pa700_probe_pose.mat', ...
                    'T2_pa720/26-Jan-2023-17-50_pa720_probe_pose.mat', ...
                    'T1_US/26-Jan-2023-17-39_us_probe_pose.mat'
                  };

ecm_data_path = {
                 'T2_pa720/image/', ...
                 'T2_pa720/image/', ...
                 'T2_pa720/image/'
                };

pa_data = {
            'T1_pa700/outdas.mat', ...
            'T2_pa720/outdas.mat', ...
            'T1_US/outdas.mat'
          };

load([dataFolder, probe_pos_data{expRoundId}]) 
load([dataFolder, pa_data{expRoundId}]) 

ecm_imgs = dir([dataFolder, ecm_data_path{expRoundId},'*.jpg']);
ecm_imgs = natsort(struct2table(ecm_imgs).name);

%% ********** reconstruct RCM trajectory **********
stamp = 1:41;
probe_rot = probe_pos_rec(1:3,1:3,stamp);
probe_rot(:,3,:) = probe_rot(:,3,:);       % revert z-axis

probe_trans = squeeze(probe_pos_rec(1:3,4,stamp));     % [m]
probe_trans(2,:) = probe_trans(2,:);  
probe_trans(3,:) = probe_trans(3,:);

frm0 = 10;

par = CircleFitByPratt(probe_trans(2:3,frm0:end)');
yc = par(1); zc = par(2); R = par(3);
% R = 0.0325;
% center = probe_trans(:,end) - R*probe_rot(1:3,3,end);
% yc = center(2); zc = center(3);

u = probe_trans([2, 3], frm0) - [yc; zc];
v = probe_trans([2, 3], end) - [yc; zc];
RCMAngle = angleBtwVectors(u, v);
% RCMAngle = 40;

theta0 = angleBtwVectors(u, [probe_trans(2, frm0)-yc; 0]);
% theta0 = (180 - RCMAngle)/2;

theta = linspace(theta0, theta0+RCMAngle, length(stamp));
xi = mean(probe_trans(1,:)) * ones(1,length(stamp));
yi = R*cosd(theta) + yc;
zi = R*sind(theta) + zc;

z_vec = [xi; yi; zi] - [xi(1); yc; zc];
x_vec = reshape(probe_rot(1:3,1,:), 3, length(stamp));
x_vec = mean(x_vec, 2) .* ones(size(z_vec));
y_vec = zeros(size(z_vec));
for frm = 1:length(stamp)
    z_vec(:,frm) = z_vec(:,frm)/norm(z_vec(:,frm));
    y_vec(:,frm) = cross(x_vec(:,frm), z_vec(:,frm));
end

probe_pos_recon = probe_pos_rec;
probe_pos_recon(1:3,1,:) = reshape(x_vec, 3, 1, length(stamp));
probe_pos_recon(1:3,2,:) = reshape(y_vec, 3, 1, length(stamp));
probe_pos_recon(1:3,3,:) = reshape(z_vec, 3, 1, length(stamp));
probe_pos_recon(1:3,4,:) = reshape([xi; yi; zi], 3, 1, length(stamp));

% ===== view trajectory =====
plot3(probe_trans(1,:), -probe_trans(2,:), -probe_trans(3,:), '.r', 'MarkerSize', 10)
grid on; hold on; axis equal
plot3(mean(probe_trans(1,:)), -yc, -zc, '.b', 'MarkerSize', 20);
plot3(xi, -yi, -zi, '.g', 'MarkerSize', 10);

quiver3(probe_trans(1,:), -probe_trans(2,:), -probe_trans(3,:), ...
        -squeeze(probe_rot(1,end,:))',-squeeze(probe_rot(2,end,:))',-squeeze(probe_rot(3,end,:))', 0.3);

quiver3(xi, -yi, -zi, -z_vec(1,:), -z_vec(2,:), -z_vec(3,:), 0.6);
legend('estimated probe position', 'estimated RCM center', 'RCM recon. probe position', ...
       'estimated probe z-axis', 'RCM recon. probe z-axis')


%% ********** generate volume **********
channelSpacing = 0.2539;
Fs = 27.78e6;
spdSound = 1480;
sampleSpacing = (1/Fs)*spdSound*1000;

res_lat = channelSpacing*1e-3;            % lateral resolution [m/pix]
res_axi = sampleSpacing*1e-3;             % axial resolution [m/pix]

upSampleFactor = 5;
outdas_upsampled = imresize3(outdas, ...
    [size(outdas, 1), size(outdas, 2), size(outdas, 3)*upSampleFactor]);

HEIGHT = size(outdas_upsampled, 1);
WIDTH = size(outdas_upsampled, 2);
CHANNEL = size(outdas_upsampled, 3);

[row, col] = meshgrid(1:HEIGHT, 1:WIDTH);
row = reshape(row, HEIGHT * WIDTH, 1);
col = reshape(col, HEIGHT * WIDTH, 1);

pa_pix_x = ((col-1) - WIDTH/2) * res_lat;     % lateral fov [m]
pa_pix_y = zeros(length(row), 1);             % elevational fov [m]
pa_pix_z = (row-1) * res_axi + 7*1e-3;        % axial fov [m]

ecm_pix_x = zeros(1, HEIGHT * WIDTH * CHANNEL);
ecm_pix_y = zeros(1, HEIGHT * WIDTH * CHANNEL);
ecm_pix_z = zeros(1, HEIGHT * WIDTH * CHANNEL);
pix_int = zeros(1, HEIGHT * WIDTH * CHANNEL);

% trajectory = probe_pos_rec;
trajectory = probe_pos_recon;
trajectory_upsampled = imresize3(trajectory, ...
    [size(trajectory, 1), size(trajectory, 2), size(trajectory, 3)*upSampleFactor]);

for frm = 1:size(trajectory_upsampled, 3)
    [x_tmp, y_tmp, z_tmp] = TransformPoints(trajectory_upsampled(:,:,frm), pa_pix_x, pa_pix_y, pa_pix_z);
    ecm_pix_x((frm - 1) * HEIGHT * WIDTH + 1:frm * HEIGHT * WIDTH) = x_tmp;
    ecm_pix_y((frm - 1) * HEIGHT * WIDTH + 1:frm * HEIGHT * WIDTH) = y_tmp;
    ecm_pix_z((frm - 1) * HEIGHT * WIDTH + 1:frm * HEIGHT * WIDTH) = z_tmp;
    pix_int((frm - 1) * HEIGHT * WIDTH + 1:frm * HEIGHT * WIDTH) = outdas_upsampled(:,:,frm)';
    fprintf('processing (%d/%d) frm\n', frm, size(trajectory_upsampled, 3))
end
clear x_tmp y_tmp z_tmp col row

% ===== reproject pixels to volume =====
xlimits = [min(ecm_pix_x), max(ecm_pix_x)];
ylimits = [min(ecm_pix_y), max(ecm_pix_y)];
zlimits = [min(ecm_pix_z), max(ecm_pix_z)];

scale = 256 / min([diff(xlimits), diff(ylimits), diff(zlimits)]);
VOLUME_HEIGHT = round(scale * diff(zlimits));         % axial
VOLUME_WIDTH = round(scale * diff(xlimits));          % lateral
VOLUME_CHANNEL = round(scale * diff(ylimits));        % elevational

volume_raw = zeros(VOLUME_HEIGHT, VOLUME_WIDTH, VOLUME_CHANNEL);

res_x = VOLUME_WIDTH   / diff(xlimits);
res_y = VOLUME_CHANNEL / diff(ylimits);
res_z = VOLUME_HEIGHT  / diff(zlimits);

ind_x = round((ecm_pix_x - xlimits(1)) * res_x);
ind_y = round((ecm_pix_y - ylimits(1)) * res_y);
ind_z = round((ecm_pix_z - zlimits(1)) * res_z);

ind_x(ind_x == 0) = 1;
ind_y(ind_y == 0) = 1;
ind_z(ind_z == 0) = 1;

for frm = 1:length(ind_x)
    volume_raw(ind_z(frm), ind_x(frm), ind_y(frm)) = pix_int(frm);
end

volume = volume_raw ./ max(volume_raw,[],'all');    % normalize

% ===== maximum intensity projection =====
dim = 1;
depthRange = 100:150;   % 1-26-2023 phantom 1st layer
% depthRange = 150:377;   % 1-26-2023 phantom 2nd layer
MIP = squeeze(max(volume(depthRange, :, :), [], dim));
MIP = interp2(MIP, 3, 'cubic');
MIP = imgaussfilt(MIP, 5);
MIP = imdilate(MIP, strel('disk', 6)); 

switch dim
    case 1
        imagesc(ylimits*1e3, xlimits*1e3, MIP);
        xlabel('y [mm]'); ylabel('x [mm]');
    case 2
        imagesc(ylimits*1e3, zlimits*1e3, MIP);
        xlabel('y [mm]'); ylabel('z [mm]');
    case 3
        imagesc(xlimits*1e3, zlimits*1e3, MIP);
        xlabel('x [mm]'); ylabel('z [mm]');
end

colormap hot; axis tight equal

%% ********** generate ecm overlay using per-frame probe pose estimation **********
% ===== const params =====
cam_mat = [596.5596, 0.0, 261.1265;         % camera intrinsics [m]
           0.0, 790.1609, 238.1423;
           0.0, 0.0, 1.0];
ECM_DISP_WIDTH = 640;                       % ecm display width 
ECM_DISP_HEIGHT = 480;                      % ecm display height
overlay_trans = 0.6;                        % overlay transparancy
num_frms = length(ecm_imgs)-1;
isGenVid = false;

if isGenVid
    aviObj = VideoWriter([dataFolder(1:end-1),'_ecm_overlay.avi']);
    aviObj.FrameRate = 10;
    aviObj.Quality = 100;
    open(aviObj);
end

h = figure('Position', [1920/4, 1080/4, 1080, 720]);

% trajectory = probe_pos_rec;
trajectory = probe_pos_recon;

for frm = 1:num_frms
    ecm_left_img = imread([dataFolder, ecm_data_path{expRoundId}, ecm_imgs{frm}]);
    ecm_left_img = imresize(ecm_left_img, [ECM_DISP_HEIGHT, ECM_DISP_WIDTH]);
    image(ecm_left_img); axis image off;
    hold on;
    
    % pa image vertices
    pa = outdas_upsampled(:,:,frm);
    pa_vert_reproj = reprojPAVert(ecm_left_img, pa, trajectory(:, :, frm), cam_mat);
    plot(pa_vert_reproj(1,:), pa_vert_reproj(2,:), 'xg', 'MarkerSize', 5);    
    
    % pa image overlay
    pa_reproj = reprojPA(ecm_left_img, pa, trajectory(:, :, frm), cam_mat);
    overlay = imagesc(pa_reproj);
    overlay.AlphaData = (overlay_trans*(pa_reproj<-37)+(pa_reproj>=-37)).*(pa_reproj ~= -9999);
    colormap hot

    hold off;
    if isGenVid
        writeVideo(aviObj, getframe(h));
    end
    pause(0.001);
end

if isGenVid
    close(aviObj);
end
