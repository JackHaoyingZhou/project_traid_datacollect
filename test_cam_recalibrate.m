%%
clc; clear; close all

num_key_pnts = 4;
num_frms = 2;
imagePoints = zeros(num_key_pnts, 2, num_frms);
worldPoints = zeros(num_key_pnts, 2);

img_key_pnts = [1209, 801;  % 1
                924, 797;   % 2
                1202, 743   % 3
                927, 742];  % 4

% [mm]
mk_width = 3;
mk_dist_len = 33;
mk_dist_width = 7;
wld_key_pnts = [mk_width+mk_dist_len, mk_width;
                mk_width, mk_width;
                mk_width+mk_dist_len, mk_width+mk_dist_width;
                mk_width, mk_width+mk_dist_width];


imagePoints(:,:,1) = img_key_pnts;
imagePoints(:,:,2) = img_key_pnts;

worldPoints = wld_key_pnts;

cameraParams = estimateCameraParameters(imagePoints, worldPoints)
