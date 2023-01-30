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

% R = 0.0325;
% R = 0.045;
% center = probe_trans(:,end) - R*probe_rot(1:3,3,end);
% yc = center(2); zc = center(3);

par = CircleFitByPratt(probe_trans(2:3,frm0:end)');
yc = par(1); zc = par(2); R = par(3);

u = probe_trans([2, 3], frm0) - [yc; zc];
v = probe_trans([2, 3], end) - [yc; zc];
RCMAngle = angleBtwVectors(u, v);
% RCMAngle = 40;

theta0 = angleBtwVectors(u, [probe_trans(2, frm0)-yc; 0]);
% theta0 = (180 - RCMAngle)/2;

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
plot3(probe_trans(1,:), -probe_trans(2,:), -probe_trans(3,:), '.r', 'MarkerSize', 10)
grid on; hold on; axis equal
plot3(mean(probe_trans(1,:)), -yc, -zc, '.b', 'MarkerSize', 20);
plot3(xi, -yi, -zi, '.g', 'MarkerSize', 10);

quiver3(probe_trans(1,:), -probe_trans(2,:), -probe_trans(3,:), ...
        -squeeze(probe_rot(1,end,:))',-squeeze(probe_rot(2,end,:))',-squeeze(probe_rot(3,end,:))', 0.3);

quiver3(xi, -yi, -zi, -z_vec(1,:), -z_vec(2,:), -z_vec(3,:), 0.6);
% legend('estimated probe position', 'estimated RCM center', 'RCM recon. probe position', ...
%        'estimated probe z-axis', 'RCM recon. probe z-axis')


%% ********** generate volume **********
channelSpacing = 0.2539;
Fs = 27.78e6;
spdSound = 1480;
sampleSpacing = (1/Fs)*spdSound*1000;

res_lat = channelSpacing*1e-3;            % lateral resolution [m/pix]
res_axi = sampleSpacing*1e-3;             % axial resolution [m/pix]

SamplingFactor = 5;
outdas = imresize3(outdas(:,:,:),[800 128 41*SamplingFactor]);

HEIGHT = size(outdas, 1);
WIDTH = size(outdas, 2);
CHANNEL = size(outdas, 3);

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
trajectory = imresize3(trajectory(:,:,:),[4 4 41*SamplingFactor]);

for frm = 1:CHANNEL
    % store pixel locations
    [x_tmp, y_tmp, z_tmp] = TransformPoints(trajectory(:,:,frm), pa_pix_x, pa_pix_y, pa_pix_z);
    ecm_pix_x((frm - 1) * HEIGHT * WIDTH + 1:frm * HEIGHT * WIDTH) = x_tmp;
    ecm_pix_y((frm - 1) * HEIGHT * WIDTH + 1:frm * HEIGHT * WIDTH) = y_tmp;
    ecm_pix_z((frm - 1) * HEIGHT * WIDTH + 1:frm * HEIGHT * WIDTH) = z_tmp;
    % store pixel intensities
    pix_int((frm - 1) * HEIGHT * WIDTH + 1:frm * HEIGHT * WIDTH) = outdas(:,:,frm)';
    fprintf('processing (%d/%d) frm\n', frm, length(ecm_imgs))
end
clear x_tmp y_tmp z_tmp col row

% ********** reproject pixels to volume **********
xlimits = [min(ecm_pix_x), max(ecm_pix_x)];
ylimits = [min(ecm_pix_y), max(ecm_pix_y)];
zlimits = [min(ecm_pix_z), max(ecm_pix_z)];

scale = 256 / min([diff(xlimits), diff(ylimits), diff(zlimits)]);
VOLUME_HEIGHT = round(scale * diff(zlimits));         % axial
VOLUME_WIDTH = round(scale * diff(xlimits));          % lateral
VOLUME_CHANNEL = round(scale * diff(ylimits));        % elevational

% VOLUME_HEIGHT = 81; % axial
% VOLUME_WIDTH = 128; % lateral
% VOLUME_CHANNEL = 600; % elevational

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

volume = volume_raw ./ max(volume_raw,[],'all');

dim = 1;
MIP = squeeze(max(volume(100:150, :, :), [], dim));
MIP = interp2(MIP, 3, 'cubic');
MIP = imgaussfilt(MIP, 5);
MIP = imdilate(MIP, strel([0 1 0; 1 1 1; 0 1 0]));
imagesc(MIP)
colormap hot; axis tight equal

switch dim
    case 1
        xlabel('y'); ylabel('x');
    case 2
        xlabel('y'); ylabel('z');
    case 3
        xlabel('x'); ylabel('z');
end

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

for i = 1:num_frms
    ecm_left_img = imread([dataFolder, ecm_data_path{expRoundId}, ecm_imgs{i}]);
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

%% ********** utilities **********
function [theta] = angleBtwVectors(u, v)

angle = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
theta = real(acosd(angle));

end

function Par = CircleFitByPratt(XY)
%--------------------------------------------------------------------------
%  
%     Circle fit by Pratt
%      V. Pratt, "Direct least-squares fitting of algebraic surfaces",
%      Computer Graphics, Vol. 21, pages 145-152 (1987)
%
%     Input:  XY(n,2) is the array of coordinates of n points x(i)=XY(i,1), y(i)=XY(i,2)
%
%     Output: Par = [a b R] is the fitting circle:
%                           center (a,b) and radius R
%
%     Note: this fit does not use built-in matrix functions (except "mean"),
%           so it can be easily programmed in any programming language
%
%--------------------------------------------------------------------------
n = size(XY,1);      % number of data points
centroid = mean(XY);   % the centroid of the data set
%     computing moments (note: all moments will be normed, i.e. divided by n)
Mxx=0; Myy=0; Mxy=0; Mxz=0; Myz=0; Mzz=0;
for i=1:n
    Xi = XY(i,1) - centroid(1);  %  centering data
    Yi = XY(i,2) - centroid(2);  %  centering data
    Zi = Xi*Xi + Yi*Yi;
    Mxy = Mxy + Xi*Yi;
    Mxx = Mxx + Xi*Xi;
    Myy = Myy + Yi*Yi;
    Mxz = Mxz + Xi*Zi;
    Myz = Myz + Yi*Zi;
    Mzz = Mzz + Zi*Zi;
end
   
Mxx = Mxx/n;
Myy = Myy/n;
Mxy = Mxy/n;
Mxz = Mxz/n;
Myz = Myz/n;
Mzz = Mzz/n;
%    computing the coefficients of the characteristic polynomial
Mz = Mxx + Myy;
Cov_xy = Mxx*Myy - Mxy*Mxy;
Mxz2 = Mxz*Mxz;
Myz2 = Myz*Myz;
A2 = 4*Cov_xy - 3*Mz*Mz - Mzz;
A1 = Mzz*Mz + 4*Cov_xy*Mz - Mxz2 - Myz2 - Mz*Mz*Mz;
A0 = Mxz2*Myy + Myz2*Mxx - Mzz*Cov_xy - 2*Mxz*Myz*Mxy + Mz*Mz*Cov_xy;
A22 = A2 + A2;
epsilon=1e-12; 
ynew=1e+20;
IterMax=20;
xnew = 0;
%    Newton's method starting at x=0
for iter=1:IterMax
    yold = ynew;
    ynew = A0 + xnew*(A1 + xnew*(A2 + 4.*xnew*xnew));
    if (abs(ynew)>abs(yold))
        disp('Newton-Pratt goes wrong direction: |ynew| > |yold|');
        xnew = 0;
        break;
    end
    Dy = A1 + xnew*(A22 + 16*xnew*xnew);
    xold = xnew;
    xnew = xold - ynew/Dy;
    if (abs((xnew-xold)/xnew) < epsilon), break, end
    if (iter >= IterMax)
        disp('Newton-Pratt will not converge');
        xnew = 0;
    end
    if (xnew<0.)
        fprintf(1,'Newton-Pratt negative root:  x=%f\n',xnew);
        xnew = 0;
    end
end
%    computing the circle parameters
DET = xnew*xnew - xnew*Mz + Cov_xy;
Center = [Mxz*(Myy-xnew)-Myz*Mxy , Myz*(Mxx-xnew)-Mxz*Mxy]/DET/2;
Par = [Center+centroid , sqrt(Center*Center'+Mz+2*xnew)];
end    %    CircleFitByPratt
