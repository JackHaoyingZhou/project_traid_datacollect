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

%%
channelSpacing = 0.2539;
Fs = 27.78e6;
spdSound = 1480;
sampleSpacing = (1/Fs)*spdSound*1000;

PAoutdas1 = outdas./max(outdas,[],'all');
PAoutdas1dB = PAoutdas1;

% Upsampling
UpSampFac = 2;

Py = channelSpacing/UpSampFac;
Pz = sampleSpacing/UpSampFac;

% Load ECM
step = size(mk0,1);

SimTraj = double([mk0,mk1,mk2,mk3,(linspace(20,-20,step)+40)']');
VesLoc = [];

% 3D matrix size
Pxx = 256;
Pyy = 256;
Pzz = 800;

ReconIMGALL = zeros(Pxx+1,Pyy+1,Pzz+1);

rotx = @(t) [1 0 0; 0 cosd(t) -sind(t) ; 0 sind(t) cosd(t)];
roty = @(t) [cosd(t) 0 sind(t) ; 0 1 0 ; -sind(t) 0  cosd(t)];
rotz = @(t) [cosd(t) -sind(t) 0 ; sind(t) cosd(t) 0 ; 0 0 1];

for Step = 1:step

    %%Search MisMark
    Mk = reshape(SimTraj(1:8,Step),2,4);
    MkNan = find(mean(Mk,1) == -1);

    if length(MkNan) == 1
        a = 1
        MkRef1 = (MkNan - 2);
        if MkRef1 <= 0
            MkRef1 = MkRef1+4;
        end
        if MkNan == 1 || MkNan == 3
            MkRef2 = MkNan + 1;
        else
            MkRef2 = MkNan - 1;
        end
        MkRef0 = find(ismember([1:4],[MkRef1 MkRef2 MkNan])==0);

        Mk(:,MkNan) = Mk(:,MkRef0) + (Mk(:,MkRef1)-Mk(:,MkRef0)) + (Mk(:,MkRef2)-Mk(:,MkRef0));
    elseif length(MkNan) == 2
        if min(MkNan == [3 4])
            Vec = Mk(:,1) - Mk(:,2);
            VecAng = atan2(Vec(1),Vec(2));
            OffSet = sqrt(sum(Vec.^2))*(6/32.5);
            Mk(:,3) = Mk(:,1)-round([OffSet*cos(VecAng) OffSet*sin(VecAng)]');
            Mk(:,4) = Mk(:,2)-round([OffSet*cos(VecAng) OffSet*sin(VecAng)]');
        elseif min(MkNan == [1 2])
            Vec = Mk(:,3) - Mk(:,4);
            VecAng = atan2(Vec(1),Vec(2));
            OffSet = sqrt(sum(Vec.^2))*(6/32.5);
            Mk(:,1) = Mk(:,3)+round([OffSet*cos(VecAng) OffSet*sin(VecAng)]');
            Mk(:,2) = Mk(:,4)+round([OffSet*cos(VecAng) OffSet*sin(VecAng)]');
        else
            pause
        end
    end

    % Find MidPoint
    Mk1 = round(mean([Mk(:,2),Mk(:,4)],2));
    Mk2 = round(mean([Mk(:,1),Mk(:,3)],2));
    Yrot = SimTraj(9,Step);
    Marker2Probe = 7; % mm
    % Set Image Overlay Param
    ProbeVec = Mk2 - Mk1;

    PAwidth = channelSpacing*128;
    PixtoMM = sqrt(sum(ProbeVec.^2))/32.5; % unit pixel/mm
    Zrot = atan2(-ProbeVec(2),ProbeVec(1))/pi*180;


    T_cam_Lap = eye(4);
    T_cam_Lap(1:3,1:3) = roty(Yrot)*rotz(Zrot); % Fake rotation

    % Rotate PA Image
    depROI = [51:250];

    % % % img = imresize(outdas(depROI,:,Step),UpSampFac);
    img = imresize(PAoutdas1dB(depROI,:,Step),UpSampFac);


    pixNum = size(img,2)*size(img,1);
    CoorsList = zeros(pixNum,3);
    Intensity = zeros(1,pixNum);
    k = 1;

    for y = 1:size(img,2)
        for z = 1:size(img,1)

            T_pix = eye(4);
            T_pix(2:3,4) = [(129-y)*Py; (z+depROI(1))*Pz];
            T_cam_pix = T_cam_Lap*T_pix;

            CoorsList(k,:) = T_cam_pix(1:3,4);
            Intensity(k) = img(z,y);

            k = k+1;
        end
    end

    ReconIMG = ones(Pxx+1,Pyy+1,Pzz+1)*(-9999);

    % Define Resolution
    Rxx = (max(CoorsList(:,1))-min(CoorsList(:,1)))/Pxx;
    Ryy = (max(CoorsList(:,2))-min(CoorsList(:,2)))/Pyy;
    Rzz = (max(CoorsList(:,3))-min(CoorsList(:,3)))/Pzz;

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
    Ixx = round((CoorsList(:,1)-min(CoorsList(:,1)))/Rxx)+1;
    Iyy = round((CoorsList(:,2)-min(CoorsList(:,2)))/Ryy)+1;
    Izz = round((CoorsList(:,3)-min(CoorsList(:,3)))/Rzz)+1;

    for i = 1:length(Intensity)
        ReconIMG(Ixx(i),Iyy(i),Izz(i)) = Intensity(i);
    end
    % ReconIMGALL = ReconIMGALL+ReconIMG;
    % ReconIMG1 = interp3(ReconIMG,0);
    ReconIMG1 = ReconIMG;
    MIP = max(ReconIMG1(:,:,:),[],3);
    % MIP = max(ReconIMGALL(:,:,:),[],3);

    PixStPt = Mk1;

    RyyP = Ryy*PixtoMM;
    RxxP = Rxx*PixtoMM;

    % % % MIP1 = MIP./max(max(MIP));
    MIP1 = MIP;

    PixLat = [1:Pyy]*RyyP+Mk1(1)+(Marker2Probe+depROI(1)*sampleSpacing)*sind(Yrot)*PixtoMM*sind(Zrot);
    PixDep = ([1:Pxx]*RxxP+Mk1(2)-(Yrot<0)*Pxx*RxxP)+(Marker2Probe+depROI(1)*sampleSpacing)*sind(Yrot)*PixtoMM;
    PixLatR(Step,:) = PixLat;
    PixDepR(Step,:) = PixDep;
    MIP2(:,:,Step) = MIP1;


    for S = 1:Step
        PixLat = PixLatR(S,:);
        PixDep = PixDepR(S,:);
        MIP3 = MIP2(:,:,S);
        MIP3(MIP3<-35) = -9999;

    end
end

%%
RED = [[0,0,0];[linspace(0.4,1,20)',linspace(0,0,20)',linspace(0,0,20)']];


Transpatency = .03;

MIP2i = [];
PixLatRi = [];
PixDepRi = [];
MIP2i = interp3(MIP2,1);
% for m = 1:257
%     for n = 1:257
%     MIP2i(m,n,:) = reshape(interp(reshape(MIP2(m,n,:),1,[]),2),1,1,[]);
%     end
% end

for m = 1:256
    PixLatRi(:,m) = interp(PixLatR(:,m),2);
    PixDepRi(:,m) = interp(PixDepR(:,m),2);
end

for S = 1:size(MIP2i,3)
    PixLat = PixLatRi(S,:);
    PixDep = PixDepRi(S,:);
    MIP3 = MIP2i(:,:,S);
    % MIP3(MIP3<0.1) = 0;
    im = imagesc(PixLat,PixDep,MIP3);
    caxis([-37 0])

    colormap(RED)
    im.AlphaData = (Transpatency*(MIP3<-37)+(MIP3>=-37)).*(MIP3 ~= -9999);
end
Transpatency = .01;

for S = 1:size(MIP2i,3)
    PixLat = PixLatRi(S,:);
    PixDep = PixDepRi(S,:);
    MIP3 = MIP2i(:,:,S);
    MIP3(MIP3<-35) = -9999;
    im = imagesc(PixLat,PixDep,MIP3);
    caxis([-37 0])

    colormap(RED)
    im.AlphaData = Transpatency*(MIP3>=-35);
end

axis([0 1920 0 1080])