% =============================================================================
% file name:    renderPA2ECM.m
% description:  render PA image to ecm view based on tracked probe pose
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

marker_trans_data = {
                    '', ...
                    '', ...
                    ''
                  };

ecm_data_path = {
                 'T1_pa700/image/', ...
                 'T2_pa720/image/', ...
                 'T1_US/image/'
                };

pa_data = {
            'T1_pa700/outdas.mat', ...
            'T2_pa720/outdas.mat', ...
            'T1_US/outdas.mat'
          };

load([dataFolder, probe_pos_data{expRoundId}]) 
load([dataFolder, pa_data{expRoundId}]) 
% outdas = 10*log10(outdas);
% outdas(outdas < -37) = -9999;

ecm_imgs = dir([dataFolder, ecm_data_path{expRoundId},'*.jpg']);
ecm_imgs = natsort(struct2table(ecm_imgs).name);

%% reconstruct RCM trajectory
stamp = 1:41;
probe_rot = probe_pos_rec(1:3,1:3,stamp);
probe_rot(:,3,:) = probe_rot(:,3,:);       % revert z-axis

probe_trans = squeeze(probe_pos_rec(1:3,4,stamp));     % [m]
probe_trans(2,:) = probe_trans(2,:);   % revert y-axis
probe_trans(3,:) = probe_trans(3,:);   % revert z-axis

frm0 = 1;
[yc,zc,R,~] = circfit(probe_trans(2,frm0:end),probe_trans(3,frm0:end));

u = probe_trans([2, 3], frm0) - [yc; zc];
v = probe_trans([2, 3], end) - [yc; zc];
RCMAngle = angleBtwVectors(u, v);
theta0 = angleBtwVectors(u, [probe_trans(2, frm0)-yc; 0]);

% % force correction
% R = 10*R;
% RCMAngle = 40;
% theta0 = 70;

% regenerate RCM trajectory
theta = linspace(theta0, theta0+RCMAngle, length(stamp));
xi = mean(probe_trans(1,:)) * ones(1,length(stamp));
% xi = probe_trans(1,:);
yi = R*cosd(theta) + yc;
zi = R*sind(theta) + zc;

z_vec = [xi; yi; zi] - [xi(1); yc; zc];
% x_vec = zeros(size(z_vec)); x_vec(1,:) = 1;
x_vec = reshape(probe_rot(1:3,1,:), 3, length(stamp));
x_vec = mean(x_vec, 2) .* ones(size(z_vec));
y_vec = zeros(size(z_vec));
for i = 1:length(stamp)
    z_vec(:,i) = z_vec(:,i)/norm(z_vec(:,i));
    y_vec(:,i) = cross(x_vec(:,i), z_vec(:,i));
end

probe_pos_recon = probe_pos_rec;
probe_pos_recon(1:3,1,:) = reshape(x_vec, 3, 1, length(stamp));
probe_pos_recon(1:3,2,:) = reshape(y_vec, 3, 1, length(stamp));
probe_pos_recon(1:3,3,:) = reshape(z_vec, 3, 1, length(stamp));
probe_pos_recon(1:3,4,:) = reshape([xi; yi; zi], 3, 1, length(stamp));

% view trajectory
plot3(probe_trans(1,:), probe_trans(2,:), probe_trans(3,:), '.r', 'MarkerSize', 10)
grid on; hold on; axis equal
plot3(mean(probe_trans(1,:)), yc, zc, '.b', 'MarkerSize', 20);
plot3(xi, yi, zi, '.g', 'MarkerSize', 10);

quiver3(probe_trans(1,:), probe_trans(2,:), probe_trans(3,:), ...
        squeeze(probe_rot(1,end,:))',squeeze(probe_rot(2,end,:))',squeeze(probe_rot(3,end,:))', 0.3);

quiver3(xi, yi, zi, z_vec(1,:), z_vec(2,:), z_vec(3,:), 0.6);
% legend('estimated probe position', 'estimated RCM center', 'RCM recon. probe position', ...
%        'estimated probe z-axis', 'RCM recon. probe z-axis')


%% generate pointcloud
channelSpacing = 0.2539;
Fs = 27.78e6;
spdSound = 1480;
sampleSpacing = (1/Fs)*spdSound*1000;

res_lat = channelSpacing*1e-3;            % lateral resolution [m/pix]
res_axi = sampleSpacing*1e-3;             % axial resolution [m/pix]

HEIGHT = size(outdas, 1);
WIDTH = size(outdas, 2);
CHANNEL = size(outdas, 3);

% [row, col] = meshgrid(HEIGHT_BEGIN:HEIGHT_BEGIN + HEIGHT - 1, ...
% WIDTH_BEGIN:WIDTH_BEGIN + WIDTH - 1);
[row, col] = meshgrid(1:HEIGHT, 1:WIDTH);
row = reshape(row, HEIGHT * WIDTH, 1);
col = reshape(col, HEIGHT * WIDTH, 1);

pa_pix_x = ((col-1) - WIDTH/2) * res_lat;     % lateral fov [m]
pa_pix_y = zeros(length(row), 1);             % elevational fov [m]
pa_pix_z = (row-1) * res_axi;                 % axial fov [m]

ecm_pix_x = zeros(1, HEIGHT * WIDTH * CHANNEL);
ecm_pix_y = zeros(1, HEIGHT * WIDTH * CHANNEL);
ecm_pix_z = zeros(1, HEIGHT * WIDTH * CHANNEL);
pix_int = zeros(1, HEIGHT * WIDTH * CHANNEL);

% trajectory = probe_pos_rec;
trajectory = probe_pos_recon;

for frm = 1:length(ecm_imgs)
    % store pixel locations
    [x_tmp, y_tmp, z_tmp] = TransformPoints(trajectory(:,:,frm), pa_pix_x, pa_pix_y, pa_pix_z);
    ecm_pix_x((frm - 1) * HEIGHT * WIDTH + 1:frm * HEIGHT * WIDTH) = x_tmp;
    ecm_pix_y((frm - 1) * HEIGHT * WIDTH + 1:frm * HEIGHT * WIDTH) = y_tmp;
    ecm_pix_z((frm - 1) * HEIGHT * WIDTH + 1:frm * HEIGHT * WIDTH) = z_tmp;
    % store pixel intensities
    pix_int((frm - 1) * HEIGHT * WIDTH + 1:frm * HEIGHT * WIDTH) = outdas(:,:,frm);
    fprintf('processing (%d/%d) frm\n', frm, length(ecm_imgs))
end
clear x_tmp y_tmp z_tmp col row

% display pointcloud
pntsRaw = [ecm_pix_x; ecm_pix_y; ecm_pix_z]';

ind_discard = pix_int < 0.02*max(pix_int);
pntsRaw(ind_discard, :) = [];
pix_int_downsampled = pix_int;
pix_int_downsampled(ind_discard) = [];

pntCloud = pointCloud(pntsRaw, 'Intensity', pix_int_downsampled');
pcshow(pntCloud);
xlabel('x'); ylabel('y'); zlabel('z'); 
axis equal tight
% make background white
set(gcf,'color','w');
set(gca,'color','w','XColor',[0.15 0.15 0.15],'YColor',[0.15 0.15 0.15],'ZColor',[0.15 0.15 0.15]);

%% generate volume
xlimits = [min(ecm_pix_x), max(ecm_pix_x)];
ylimits = [min(ecm_pix_y), max(ecm_pix_y)];
zlimits = [min(ecm_pix_z), max(ecm_pix_z)];

scale = 256 / min([diff(xlimits), diff(ylimits), diff(zlimits)]);
VOLUME_HEIGHT = round(scale * diff(zlimits));         % axial
VOLUME_WIDTH = round(scale * diff(xlimits));          % lateral
VOLUME_CHANNEL = round(scale * diff(ylimits));        % elevational

% VOLUME_HEIGHT = 256; % axial
% VOLUME_WIDTH = 256; % lateral
% VOLUME_CHANNEL = 800; % elevational

volume_raw = zeros(VOLUME_HEIGHT, VOLUME_WIDTH, VOLUME_CHANNEL);

% Define Resolution
res_x = VOLUME_WIDTH   / diff(xlimits);
res_y = VOLUME_CHANNEL / diff(ylimits);
res_z = VOLUME_HEIGHT  / diff(zlimits);

% Search for the nearest voxel in the matrix
ind_x = round((ecm_pix_x - xlimits(1)) * res_x);
ind_y = round((ecm_pix_y - ylimits(1)) * res_y);
ind_z = round((ecm_pix_z - zlimits(1)) * res_z);

ind_x(ind_x == 0) = 1;
ind_y(ind_y == 0) = 1;
ind_z(ind_z == 0) = 1;

for i = 1:length(ind_x)
    volume_raw(ind_z(i), ind_x(i), ind_y(i)) = pix_int(i);
end

% volume_raw(volume_raw < 0.1) = 0;
% volume = interp3(volume_raw,1);
volume = volume_raw;

dim = 1;
MIP = squeeze(max(volume,[],dim));
imagesc(MIP)

% topView = squeeze(volume(30,:,:));
% imagesc(topView)

% sideView = squeeze(volume(:,10,:));
% imagesc(flipud(sideView))

% frontView = volume(:,:,1);
% imagesc(frontView)

colormap hot; axis tight equal
switch dim
    case 1
        xlabel('y'); ylabel('x');
    case 2
        xlabel('y'); ylabel('z');
    case 3
        xlabel('x'); ylabel('z');
end

%% generate ecm overlay using per-frame probe pose estimation
% ********** const params **********
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

for i = 1:num_frms
    ecm_left_img = imread([[dataFolder, ecm_data_path{expRoundId}, ecm_imgs{i}]]);
    ecm_left_img = imresize(ecm_left_img, [ECM_DISP_HEIGHT, ECM_DISP_WIDTH]);
    image(ecm_left_img); axis image off;
    hold on;
    
    % pa image vertices
    pa = outdas(:,:,i);
    pa_vert_reproj = reprojPAVert(ecm_left_img, pa, trajectory(:, :, i), cam_mat);
    plot(pa_vert_reproj(1,:), pa_vert_reproj(2,:), 'xg', 'MarkerSize', 5);    
    
    % pa image overlay
    pa_reproj = reprojPA(ecm_left_img, pa, trajectory(:, :, i), cam_mat);
    overlay = imagesc(pa_reproj);
    overlay.AlphaData = (overlay_trans*(pa_reproj<-37)+(pa_reproj>=-37)).*(pa_reproj ~= -9999)
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

%% utilities
function [theta] = angleBtwVectors(u, v)

angle = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
theta = real(acosd(angle));

end

function   [xc,yc,R,a] = circfit(x,y)
%
%   [xc yx R] = circfit(x,y)
%
%   fits a circle  in x,y plane in a more accurate
%   (less prone to ill condition )
%  procedure than circfit2 but using more memory
%  x,y are column vector where (x(i),y(i)) is a measured point
%
%  result is center point (yc,xc) and radius R
%  an optional output is the vector of coeficient a
% describing the circle's equation
%
%   x^2+y^2+a(1)*x+a(2)*y+a(3)=0
%
%  By:  Izhak bucher 25/oct /1991, 
x=x(:); y=y(:);
a=[x y ones(size(x))]\[-(x.^2+y.^2)];
xc = -.5*a(1);
yc = -.5*a(2);
R  =  sqrt((a(1)^2+a(2)^2)/4-a(3));

end
