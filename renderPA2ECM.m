% =============================================================================
% file name:    renderPA2ECM.m
% description:  render PA image to ecm view based on tracked probe pose
% author:       Xihan Ma
% date:         2022-12-30
% =============================================================================
clc; clear; close all;
addpath('utils/');

%% load data
dataFolder = 'probe_pose_recording/';
dataID = 2;
probe_pose_data = {
                    '26-Jan-2023-17-27_paT700_probe_pose.mat', ...
                    '26-Jan-2023-17-50_paT720_probe_pose.mat', ...
                    '26-Jan-2023-17-39_us_probe_pose.mat'
                  };

marker_pos_data = {
                    '', ...
                    '', ...
                    ''
                  };

load([dataFolder, probe_pose_data{dataID}]) 

% if ~exist('mk_trans_avg_rec', 'var')
%     load('probe_pose_recording/26-Jan-2023-13-23_mk_trans.mat')
% end

if ~exist('outdas', 'var')
    load('outdas.mat')
end

%% plot probe trajectory
stamp = 1:40;
probe_rot = probe_pos_rec(1:3,1:3,stamp);
probe_rot(:,3,:) = -probe_rot(:,3,:);       % revert z-axis

probe_pos = squeeze(probe_pos_rec(1:3,4,stamp)).*1e3;
probe_pos(2,:) = -probe_pos(2,:);   % revert y-axis
probe_pos(3,:) = -probe_pos(3,:);   % revert marker depth

plot3(probe_pos(1,:), probe_pos(2,:), probe_pos(3,:), '.g', 'MarkerSize', 10)
xlim([-0.05 0.05]*1e3);
ylim([-0.1 0.1]*1e3);
% zlim([0 0.3]*1e3)
grid on; hold on; axis equal

% z-axis direction
for i = 1:length(probe_pos)
    quiver3(probe_pos(1,i),probe_pos(2,i),probe_pos(3,i), ...
            probe_rot(1,end,i),probe_rot(2,end,i),probe_rot(3,end,i), 0.2e1);
end

xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]')

%% generate ecm overlay


