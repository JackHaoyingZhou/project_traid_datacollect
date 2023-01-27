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

ecm_imgs = dir([dataFolder, ecm_data_path{expRoundId},'*.jpg']);
ecm_imgs = natsort(struct2table(ecm_imgs).name);

%% plot probe trajectory
stamp = 1:40;
probe_rot = probe_pos_rec(1:3,1:3,stamp);
probe_rot(:,3,:) = -probe_rot(:,3,:);       % revert z-axis

probe_trans = squeeze(probe_pos_rec(1:3,4,stamp)).*1e3;     % [mm]
probe_trans(2,:) = -probe_trans(2,:);   % revert y-axis
probe_trans(3,:) = -probe_trans(3,:);   % revert marker depth

plot3(probe_trans(1,:), probe_trans(2,:), probe_trans(3,:), '.g', 'MarkerSize', 10)
xlim([-0.05 0.05]*1e3);
ylim([-0.1 0.1]*1e3);
% zlim([0 0.3]*1e3)
grid on; hold on; axis equal

% z-axis direction
for i = 1:length(probe_trans)
    quiver3(probe_trans(1,i),probe_trans(2,i),probe_trans(3,i), ...
            probe_rot(1,end,i),probe_rot(2,end,i),probe_rot(3,end,i), 0.2e1);
end

xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]')


%% estimate RCM params
frm0 = 10;
[yc,zc,R,~] = circfit(probe_trans(2,frm0:end),probe_trans(3,frm0:end))

% u = probe_rot(1:3, end, 1);
% v = probe_rot(1:3, end, stamp(end));

u = probe_trans([2, 3], frm0) - [yc; zc];
v = probe_trans([2, 3], end) - [yc; zc];

angle = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
RCMAngle = real(acosd(angle))

% regenerate RCM trajectory
theta = linspace(-RCMAngle/2, RCMAngle/2, length(stamp));
yi = R*sind(theta/2) + yc;
zi = -R*cosd(theta/2) + zc;

plot3(probe_trans(1,:), probe_trans(2,:), probe_trans(3,:), '.r', 'MarkerSize', 10)
grid on; hold on; axis equal

plot3(mean(probe_trans(1,:)), yc, zc, '.b', 'MarkerSize', 20);
plot3(mean(probe_trans(1,:))*ones(1,length(stamp)), yi, zi, '.g', 'MarkerSize', 10);

%% generate PA volume
channelSpacing = 0.2539;
Fs = 27.78e6;
spdSound = 1480;
sampleSpacing = (1/Fs)*spdSound*1000;

res_x = channelSpacing*1e-3;            % lateral resolution [m/pix]
res_z = sampleSpacing*1e-3;             % axial resolution [m/pix]

HEIGHT = size(outdas, 1);
WIDTH = size(outdas, 2);
CHANNEL = size(outdas, 3);

% [row, col] = meshgrid(HEIGHT_BEGIN:HEIGHT_BEGIN + HEIGHT - 1, ...
% WIDTH_BEGIN:WIDTH_BEGIN + WIDTH - 1);
[row, col] = meshgrid(1:HEIGHT, 1:WIDTH);
row = reshape(row, HEIGHT * WIDTH, 1);
col = reshape(col, HEIGHT * WIDTH, 1);

pa_pix_x = ((col-1) - WIDTH/2) * res_x;     % lateral fov [m]
pa_pix_y = zeros(length(row), 1);           % elevational fov [m]
pa_pix_z = (row-1) * res_z;                 % axial fov [m]

ecm_pix_xyz = zeros(3, HEIGHT * WIDTH * CHANNEL);
pix_int = zeros(1, HEIGHT * WIDTH * CHANNEL);

for frm = 1:length(ecm_imgs)-1
    % calculate pixel locations
    ecm_pix_xyz_tmp = TransformPoints(probe_pos_rec(frm), pa_pix_x, pa_pix_y, pa_pix_z);
    ecm_pix_xyz((frm - 1) * HEIGHT * WIDTH + 1:frm * HEIGHT * WIDTH) = ecm_pix_xyz_tmp;
    % calculate pixel intensities
    pix_int((frm - 1) * HEIGHT * WIDTH + 1:frm * HEIGHT * WIDTH) = outdas(:,:,frm);
    fprintf('processing (%d/%d) frm\n', frm, length(ecm_imgs)-1)
end

% depROI = 51:250;

VOLUME_HEIGHT = 800;
VOLUME_WIDTH = 256;
VOLUME_CHANNEL = 256;
volume_raw = zeros(VOLUME_HEIGHT, VOLUME_WIDTH, VOLUME_CHANNEL);

% Define Resolution
Rxx = (max(pix_ecm_xyz(1,:))-min(pix_ecm_xyz(1,:)))/VOLUME_WIDTH;
Ryy = (max(pix_ecm_xyz(2,:))-min(pix_ecm_xyz(2,:)))/VOLUME_CHANNEL;
Rzz = (max(pix_ecm_xyz(3,:))-min(pix_ecm_xyz(3,:)))/VOLUME_HEIGHT;

if Rxx == 0
    Rxx = 1e-10;
end
if Ryy == 0
    Ryy = 1e-10;
end
if Rzz == 0
    Rzz = 1e-10;
end

% Search for the nearest voxel in the matrix
Ixx = round((pix_ecm_xyz(1,:)-min(pix_ecm_xyz(1,:)))/Rxx)+1;
Iyy = round((pix_ecm_xyz(2,:)-min(pix_ecm_xyz(2,:)))/Ryy)+1;
Izz = round((pix_ecm_xyz(3,:)-min(pix_ecm_xyz(3,:)))/Rzz)+1;

for i = 1:length(pix_int)
    volume_raw(Ixx(i),Iyy(i),Izz(i)) = pix_int(i);
end

%% view MIP
dim = 1;
MIP = squeeze(max(volume_raw(:,:,:),[],dim));
imagesc(MIP); colormap hot; axis image

%% generate ecm overlay using per-frame probe pose estimation
% ********** const params **********
cam_mat = [596.5596, 0.0, 261.1265;         % camera intrinsics [m]
           0.0, 790.1609, 238.1423;
           0.0, 0.0, 1.0];
ECM_DISP_WIDTH = 640;                       % ecm display width 
ECM_DISP_HEIGHT = 480;                      % ecm display height
overlay_trans = 0.6;                        % overlay transparancy
num_frms = length(ecm_imgs)-1;
isGenVid = true;

if isGenVid
    aviObj = VideoWriter([dataFolder(1:end-1),'_ecm_overlay.avi']);
    aviObj.FrameRate = 10;
    aviObj.Quality = 100;
    open(aviObj);
end

h = figure('Position', [1920/4, 1080/4, 1080, 720]);
for i = 1:num_frms
    ecm_left_img = imread([[dataFolder, ecm_data_path{expRoundId}, ecm_imgs{i}]]);
    ecm_left_img = imresize(ecm_left_img, [ECM_DISP_HEIGHT, ECM_DISP_WIDTH]);
    image(ecm_left_img); axis image off;
    hold on;
    
    % pa image vertices
    pa_vert_reproj = reprojPAVert(ecm_left_img, pa, probe_pos_rec(:, :, i), cam_mat);
    plot(pa_vert_reproj(1,:), pa_vert_reproj(2,:), 'xg', 'MarkerSize', 5);    
    
    % pa image overlay
    pa = outdas(:,:,i);
    pa_reproj = reprojPA(ecm_left_img, pa, probe_pos_rec(:, :, i), cam_mat);
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

%% utility
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
