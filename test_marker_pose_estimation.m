clear all
close all
clc

f1 = figure;
f1.Position = [810 55 1735 931];

channelSpacing = 0.2539;
Fs = 27.78e6;
spdSound = 1480;
sampleSpacing = (1/Fs)*spdSound*1000;

CaseName = ['T10_PA850'];
CaseName1 = [CaseName(1:3),'_pa',CaseName(7:end)];
CaseName1 = [CaseName(1:3),'_US'];
% CaseName1 = CaseName;
load(['PA_data/',CaseName,'/outdas.mat'],'outdas');
load(['daVinci_data/',CaseName1,'_4markers/labeled/mk_pix.mat'])    % XM: changed all '\' to '/' to run on mac and linux

outdas = outdas(1:350,:,:);
PAoutdas1 = outdas./max(outdas,[],'all');

%% test marker based rendering
% ========== marker traj estimation ==========
cam_mat = [1811.1, 0.0, 813.3;
           0.0, 1815.3, 781.8;
           0.0, 0.0, 1.0];
       
mk0_uv = double([mk0, ones(size(mk0,1),1)]);
mk1_uv = double([mk1, ones(size(mk1,1),1)]);
mk2_uv = double([mk2, ones(size(mk2,1),1)]);
mk3_uv = double([mk3, ones(size(mk3,1),1)]);

mk0_uv(mk0_uv == -1) = nan;
mk1_uv(mk1_uv == -1) = nan;
mk2_uv(mk2_uv == -1) = nan;
mk3_uv(mk3_uv == -1) = nan;

mk_diff = sqrt((mk0_uv(:,1)-mk1_uv(:,1)).^2 + (mk0_uv(:,2)-mk1_uv(:,2)).^2);
scale = mk_diff/0.032;

mk0_xyz = cam_mat \ mk0_uv';
mk1_xyz = cam_mat \ mk1_uv';
mk2_xyz = cam_mat \ mk2_uv';
mk3_xyz = cam_mat \ mk3_uv';

mk0_xyz(3,:) = mk0_xyz(3,:).*(scale');
mk1_xyz(3,:) = mk1_xyz(3,:).*(scale');
mk2_xyz(3,:) = mk2_xyz(3,:).*(scale');
mk3_xyz(3,:) = mk3_xyz(3,:).*(scale');

% figure();
% plot3(mk0_xyz(1,:), mk0_xyz(2,:), mk0_xyz(3,:), '.b','MarkerSize',20);
% hold on
% plot3(mk1_xyz(1,:), mk1_xyz(2,:), mk1_xyz(3,:), '.k','MarkerSize',20);
% plot3(mk2_xyz(1,:), mk2_xyz(2,:), mk2_xyz(3,:), '.r','MarkerSize',20);
% plot3(mk3_xyz(1,:), mk3_xyz(2,:), mk3_xyz(3,:), '.g','MarkerSize',20);
% grid on
% xlabel('x'); ylabel('y'); zlabel('z')
% legend('mk0', 'mk1', 'mk2', 'mk3')

% ========== test rendering based on estimated marker traj ==========
startStamp = 16;
figure('Position', [1920/4, 1080/4, 640, 480]);
for i = startStamp:size(mk0,1)
    plot3(mk0_xyz(1,i), mk0_xyz(2,i), mk0_xyz(3,i), '.b','MarkerSize',20);
    hold on
    plot3(mk1_xyz(1,i), mk1_xyz(2,i), mk1_xyz(3,i), '.k','MarkerSize',20);
    plot3(mk2_xyz(1,i), mk2_xyz(2,i), mk2_xyz(3,i), '.r','MarkerSize',20);
    plot3(mk3_xyz(1,i), mk3_xyz(2,i), mk3_xyz(3,i), '.g','MarkerSize',20);
    hold off
    grid on
    xlim([-0.4, 0.4]); ylim([-0.35, -0.0]); zlim([1.35, 1.42]*1e4)
    xlabel('x'); ylabel('y'); zlabel('z')
    legend('mk0', 'mk1', 'mk2', 'mk3')
    pause(0.02);
end

%% ROI
PAoutdas1(105:165,:,26:27) = PAoutdas1(105:165,:,26:27)*1.5;
PAoutdas1(:,110:128,:) = PAoutdas1(:,110:128,:)*.5;

R = [1 7 10 15 20 25 26 28 35 41];
D = [170 160 150 130 130 140 105 140 120 120]-10;
F = 0.2;

for e = 1:length(R)-1
    
    PAoutdas1(1:D(e),1:80,R(e):R(e+1)) = PAoutdas1(1:D(e),1:80,R(e):R(e+1))*F;
    PAoutdas1(D(e)+50:end,1:80,R(e):R(e+1)) = PAoutdas1(D(e)+50:end,1:80,R(e):R(e+1))*F;
    PAoutdas1(1:D(e)+30,81:128,R(e):R(e+1)) = PAoutdas1(1:D(e)+30,81:128,R(e):R(e+1))*F;
    PAoutdas1(D(e)+80:end,81:128,R(e):R(e+1)) = PAoutdas1(D(e)+80:end,81:128,R(e):R(e+1))*F;

end
%%
PAoutdas1dB = db(PAoutdas1);

for i = 1:size(PAoutdas1,3)
    temp = smoothAblation(PAoutdas1dB(:,:,i),1);
    PAoutdas1dB(2:349,2:127,i) = temp(2:end,2:end);
end

% outdas = db(outdas);

%% Upsampling
UpSampFac = 2;

Py = channelSpacing/UpSampFac;
Pz = sampleSpacing/UpSampFac;

%% Load ECM
step = size(mk0,1);

SimTraj = double([mk0,mk1,mk2,mk3,(linspace(20,-20,step)+40)']');
VesLoc = [];

%% Video Recording
% writerObj = VideoWriter('Test.avi');
% writerObj.FrameRate = 10;
% open(writerObj);

%%
% 3D matrix size
Pxx = 256;
Pyy = 256;
Pzz = 800;

ReconIMGALL = zeros(Pxx+1,Pyy+1,Pzz+1);

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

%% Find MidPoint
Mk1 = round(mean([Mk(:,2),Mk(:,4)],2));
Mk2 = round(mean([Mk(:,1),Mk(:,3)],2));
Yrot = SimTraj(9,Step);
Marker2Probe = 7; % mm
%% Set Image Overlay Param
ProbeVec = Mk2 - Mk1;

PAwidth = channelSpacing*128;
PixtoMM = sqrt(sum(ProbeVec.^2))/32.5; % unit pixel/mm
Zrot = atan2(-ProbeVec(2),ProbeVec(1))/pi*180;
Transpatency = .6;
% Transpatency = 50;

T_cam_Lap = eye(4);
T_cam_Lap(1:3,1:3) = roty(Yrot)*rotz(Zrot); % Fake rotation

%% Rotate PA Image
% if Step <= 5
%     depROI = [175:200];
% elseif Step >= 12 && Step <= 28
%     depROI = [100:175];
% else
%     depROI = [150:200];
% end
depROI = [51:250];

% % % img = imresize(outdas(depROI,:,Step),UpSampFac);
img = imresize(PAoutdas1dB(depROI,:,Step),UpSampFac);

ECM = imread(['daVinci_data\',CaseName1,'_2markers\labeled\',num2str(Step),'_labeled.png']);

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
image(ECM)
axis image
hold on
plot([Mk1(1),Mk2(1)],[Mk1(2),Mk2(2)],'m','LineWidth',2)
% plot([PixStPt(1),PixStPt(2)],[PixStPt(3),PixStPt(4)],'m','LineWidth',2)
PixLat = [1:Pyy]*RyyP+Mk1(1)+(Marker2Probe+depROI(1)*sampleSpacing)*sind(Yrot)*PixtoMM*sind(Zrot);
PixDep = ([1:Pxx]*RxxP+Mk1(2)-(Yrot<0)*Pxx*RxxP)+(Marker2Probe+depROI(1)*sampleSpacing)*sind(Yrot)*PixtoMM;
im = imagesc(PixLat,PixDep,MIP1);
colormap hot
caxis([-37 0])
im.AlphaData = (Transpatency*(MIP1<-37)+(MIP1>=-37)).*(MIP1 ~= -9999);
% im.AlphaData = Transpatency*MIP1;

PixLatR(Step,:) = PixLat;
PixDepR(Step,:) = PixDep;
MIP2(:,:,Step) = MIP1;


for S = 1:Step
PixLat = PixLatR(S,:);
PixDep = PixDepR(S,:);
MIP3 = MIP2(:,:,S);
MIP3(MIP3<-35) = -9999;
% MIP3(MIP3<-40) = 0;
im = imagesc(PixLat,PixDep,MIP3);

colormap hot
im.AlphaData = Transpatency*(MIP3>=-35);
end


axis([0 1920 0 1080])

hold off
drawnow

% frame=getframe(gcf);
% writeVideo(writerObj, frame);
end

% close(writerObj);
pause
%%
RED = [[0,0,0];[linspace(0.4,1,20)',linspace(0,0,20)',linspace(0,0,20)']];
image(ECM)
axis image
hold on

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