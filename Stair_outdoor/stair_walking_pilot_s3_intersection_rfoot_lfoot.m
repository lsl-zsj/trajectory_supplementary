function stair_walking_pilot_s3_intersection_rfoot_lfoot()
% two feet trajectory estimation of square walking: yes
close all
clear all
addpath(genpath('D:\Work\Task\IMU_Code\IMU-Matlab'));
addpath(genpath('.\data'));
load('IMU_STAIR_3.mat')
%% 
rIMU=IMU.right;
% obtain the orientation
fs=rIMU.fs;
rAccelerometer=-rIMU.Acceleration;
rGyroscope=rIMU.Gyroscope;
rMagnetic=rIMU.Magnetic*45;
rlen=length(rAccelerometer);
for i=1:rlen
    rMag_norm(i)=norm(rMagnetic(i,:));
    rAcc_norm(i)=norm(rAccelerometer(i,:));
end
rFreeAcc=rIMU.FreeAcc;
%% right foot orientation
%% CEKS
sigma_acc_init=2.3;
sigma_mag_init=1.7*0.5;
sigma_acc=sigma_acc_init;
sigma_mag=sigma_mag_init;
rt=0:1/fs:1/fs*(rlen-1);
stdGyro = 0.001*5;                % (rad/s)
stdAcc = 0.0981;           % (g)
stdMag  = 0.02;          % (a.u.)
%reset_index= [];
[cekf,q1] = SAB_New_MKMC(rIMU.Acceleration, rIMU.Gyroscope, rIMU.Magnetic, rt, stdAcc, stdGyro, stdMag, sigma_acc,sigma_mag);
for i=1:length(q1)
    rQuat_cekf(i)=quaternion(q1(i,4),q1(i,1),q1(i,2),q1(i,3));
end
reuler_cekf=eulerd(rQuat_cekf,'ZXY','frame');
%% smoother
ekfb=SAB_New_MKMCS(cekf,rIMU.Acceleration,rIMU.Magnetic);
q1=ekfb.stateb';
rQuat_ceks=rQuat_cekf;
for i=1:length(q1)
    rQuat_ceks(i)=quaternion(q1(i,4),q1(i,1),q1(i,2),q1(i,3));
end
reuler_ceks=eulerd(rQuat_ceks,'ZXY','frame');
%% right free acceleration
g=[0,0,9.81]; 
acc=rAccelerometer;
acc_q=quaternion([zeros(length(acc),1),acc]);
% ceks
g_se=rQuat_ceks'; % 
%acc_es=g_se.*acc_q.*conj(g_se); % obtain the acc readings in the inertial frame
acc_es=conj(g_se).*acc_q.*(g_se); % obtain the acc readings in the inertial frame
acc_esv = compact(acc_es);
acc_es=acc_esv(:,2:4);
acc_free=acc_es-g; % free acceleration in the initial frame
rfreeAcc_ceks=acc_free;
%% segmentation of right foot
clear rpos rpos_1N rP_1n
mydata=[rt',rAccelerometer,rGyroscope,rMagnetic];
[P,M]=gyroscope_norm(mydata,rfreeAcc_ceks,1:length(rAccelerometer),fs); % the moveing interval except the first row
len=length(rt);
rvel=zeros(len,3);
rPvm=zeros(len,1);
%% trajectory estimation: loop
[seg,col]=size(P);
for i=1:seg-1
    index=P(i,2):P(i+1,1);
    acc=rfreeAcc_ceks(index,:);
    v_init=zeros(3,1);
    % forward velocity estimation
    VEL=velocity_estimation(acc,v_init,4*stdAcc,index);
    % forward position estimation
    % store the vel and Pv
    rvel(index,:)=VEL.vel;
    rPvm(index,:)=VEL.Pv; % velocity with covariance
end
% position estimation
p_init=zeros(3,1);
rPOSF=position_estimation(rvel,p_init,rPvm);
rposf=rPOSF.pos;
rPposf=rPOSF.Ppm;
% backward
rPOSB=position_estimation_b(rvel,p_init,rPvm);
rposb=rPOSB.pos;
rPposb=rPOSB.Ppm;
% fusion
[rpos,rK,rPP]=position_fusion(rposf,rposb,rPposf,rPposb);
rpos_1N=rpos;
rP_1n=rPP;
%% assume that we know the point of M
% position M: (0,0,-3.6)
clear rpos_1N_seg rP_1n_segc
%% seg=seg1+seg2
index_seg1=1:M;
index_seg2=M+1:len;
posm=[0,0,-3.6];
% position estimation： seg1
p_init=zeros(3,1);
rPOSF_seg1=position_estimation(rvel(index_seg1,:),p_init,rPvm(index_seg1,:));
rposf_seg1=rPOSF_seg1.pos;
rPposf_seg1=rPOSF_seg1.Ppm;
% backward
p_init=posm';
rPOSB_seg1=position_estimation_b(rvel(index_seg1,:),p_init,rPvm(index_seg1,:));
rposb_seg1=rPOSB_seg1.pos;
rPposb_seg1=rPOSB_seg1.Ppm;
% fusion
[rpos_seg1,rK_seg1,rPP_seg1]=position_fusion(rposf_seg1,rposb_seg1,rPposf_seg1,rPposb_seg1);
rpos_1N_seg1=rpos_seg1;
rP_1n_seg1=rPP_seg1;

% position estimation： seg2
p_init=posm';
rPOSF_seg2=position_estimation(rvel(index_seg2,:),p_init,rPvm(index_seg2,:));
rposf_seg2=rPOSF_seg2.pos;
rPposf_seg2=rPOSF_seg2.Ppm;
% backward
p_init=zeros(3,1);
rPOSB_seg2=position_estimation_b(rvel(index_seg2,:),p_init,rPvm(index_seg2,:));
rposb_seg2=rPOSB_seg2.pos;
rPposb_seg2=rPOSB_seg2.Ppm;
% fusion
[rpos_seg2,rK_seg2,rPP_seg2]=position_fusion(rposf_seg2,rposb_seg2,rPposf_seg2,rPposb_seg2);
rpos_1N_seg2=rpos_seg2;
rP_1n_seg2=rPP_seg2;

% piece together
rpos_1N_seg=zeros(len,3);
rP_1n_seg=zeros(len,1);
rpos_1N_seg(index_seg1,:)=rpos_1N_seg1;
rpos_1N_seg(index_seg2,:)=rpos_1N_seg2;
rP_1n_seg(index_seg1,:)=rP_1n_seg1;
rP_1n_seg(index_seg2,:)=rP_1n_seg2;

%% store
rQuat_ceks_s=rQuat_ceks;
reuler_ceks_s=reuler_ceks;
rposf_s=rposf;
rpos_1N_seg_s=rpos_1N_seg;

figure
x1=subplot(3,1,1);
hold on
plot(reuler_ceks(:,1),'linewidth',0.6)
plot(rIMU.EulerNew(:,1),'linewidth',0.6)
legend('EKS','Xsens')
x2=subplot(3,1,2);
hold on
plot(reuler_ceks(:,2),'linewidth',0.6)
x3=subplot(3,1,3);
hold on
plot(reuler_ceks(:,3),'linewidth',0.6)
linkaxes([x1,x2,x3],'x')

figure
hold on
plot3(rposf(:,1),rposf(:,2),rposf(:,3),'LineWidth',2,'color','g')
plot3(rpos_1N(:,1),rpos_1N(:,2),rpos_1N(:,3),'LineWidth',2,'color','blue')
plot3(rpos_1N_seg(:,1),rpos_1N_seg(:,2),rpos_1N_seg(:,3),'LineWidth',2,'color','red')
scatter3(rpos_1N_seg(M,1),rpos_1N_seg(M,2),rpos_1N_seg(M,3),100,'o','filled')
scatter3(rpos_1N_seg(1,1),rpos_1N_seg(1,2),rpos_1N_seg(1,3),200,'filled','pentagram')
xlabel('x (m)','interpreter','latex')
ylabel('y (m)','interpreter','latex')
zlabel('z (m)','interpreter','latex')
view(-100,-8)
legend('posf','posb','pos fusion','S','M','interpreter','latex','Location','southwest')
set(gca,'fontsize',16)
set(gcf,'position',[100 100 750 600])



% figure
% hold on
% x1=subplot(2,1,1);
% plot(rMag_norm,'linewidth',2,'color','red')
% ylabel('Mag ($\mu T$)','interpreter','latex')
% legend('Mag norm')
% xticks([])
% ylim([10 100])
% set(gca,'fontsize',18)
% x2=subplot(2,1,2);
% plot(rAcc_norm,'linewidth',2,'color','red')
% legend('Acc norm')
% xlabel('time (s)','Interpreter','latex')
% ylabel('Acc ($m/s^2$)','interpreter','latex')
% set(gca,'fontsize',18)
% linkaxes([x1,x2],'x')


%% %% %% %% %%  left foot %% %% %% %% %%

%% 
rIMU=IMU.left;
% obtain the orientation
fs=rIMU.fs;
rAccelerometer=-rIMU.Acceleration;
rGyroscope=rIMU.Gyroscope;
rMagnetic=rIMU.Magnetic*45;
rlen=length(rAccelerometer);
for i=1:rlen
    rMag_norm(i)=norm(rMagnetic(i,:));
    rAcc_norm(i)=norm(rAccelerometer(i,:));
end
rFreeAcc=rIMU.FreeAcc;
%% left foot orientation
%% CEKS
sigma_acc_init=2.3;
sigma_mag_init=1.7;
sigma_acc=sigma_acc_init;
sigma_mag=sigma_mag_init;
rt=0:1/fs:1/fs*(rlen-1);
stdGyro = 0.001*5;                % (rad/s)
stdAcc = 0.0981;           % (g)
stdMag  = 0.02;          % (a.u.)
%reset_index= [];
[cekf,q1] = SAB_New_MKMC(rIMU.Acceleration, rIMU.Gyroscope, rIMU.Magnetic, rt, stdAcc, stdGyro, stdMag, sigma_acc,sigma_mag);
for i=1:length(q1)
    rQuat_cekf(i)=quaternion(q1(i,4),q1(i,1),q1(i,2),q1(i,3));
end
reuler_cekf=eulerd(rQuat_cekf,'ZXY','frame');
%% smoother
ekfb=SAB_New_MKMCS(cekf,rIMU.Acceleration,rIMU.Magnetic);
q1=ekfb.stateb';
rQuat_ceks=rQuat_cekf;
for i=1:length(q1)
    rQuat_ceks(i)=quaternion(q1(i,4),q1(i,1),q1(i,2),q1(i,3));
end
reuler_ceks=eulerd(rQuat_ceks,'ZXY','frame');

figure
x1=subplot(3,1,1);
hold on
plot(reuler_ceks(:,1),'linewidth',0.6)
plot(rIMU.EulerNew(:,1)+37,'linewidth',0.6)
legend('EKS','Xsens')
x2=subplot(3,1,2);
hold on
plot(reuler_ceks(:,2),'linewidth',0.6)
x3=subplot(3,1,3);
hold on
plot(reuler_ceks(:,3),'linewidth',0.6)
linkaxes([x1,x2,x3],'x')

figure
hold on
x1=subplot(2,1,1);
plot(rMag_norm,'linewidth',2,'color','red')
ylabel('Mag ($\mu T$)','interpreter','latex')
legend('Mag norm')
xticks([])
ylim([10 150])
set(gca,'fontsize',18)
x2=subplot(2,1,2);
plot(rAcc_norm,'linewidth',2,'color','red')
legend('Acc norm')
xlabel('time (s)','Interpreter','latex')
ylabel('Acc ($m/s^2$)','interpreter','latex')
set(gca,'fontsize',18)
linkaxes([x1,x2],'x')
%% left free acceleration
g=[0,0,9.81]; 
acc=rAccelerometer;
acc_q=quaternion([zeros(length(acc),1),acc]);
% ceks
g_se=rQuat_ceks'; % 
%acc_es=g_se.*acc_q.*conj(g_se); % obtain the acc readings in the inertial frame
acc_es=conj(g_se).*acc_q.*(g_se); % obtain the acc readings in the inertial frame
acc_esv = compact(acc_es);
acc_es=acc_esv(:,2:4);
acc_free=acc_es-g; % free acceleration in the initial frame
rfreeAcc_ceks=acc_free;
%% segmentation of right foot
clear rpos rpos_1N rP_1n
mydata=[rt',rAccelerometer,rGyroscope,rMagnetic];
[P,M]=gyroscope_norm_r(mydata,rfreeAcc_ceks,1:length(rAccelerometer),fs); % the moveing interval except the first row
len=length(rt);
rvel=zeros(len,3);
rPvm=zeros(len,1);
%% trajectory estimation: loop
[seg,col]=size(P);
for i=1:seg-1
    index=P(i,2):P(i+1,1);
    acc=rfreeAcc_ceks(index,:);
    v_init=zeros(3,1);
    % forward velocity estimation
    VEL=velocity_estimation(acc,v_init,4*stdAcc,index);
    % forward position estimation
    % store the vel and Pv
    rvel(index,:)=VEL.vel;
    rPvm(index,:)=VEL.Pv; % velocity with covariance
end
% position estimation
pxx=0;
pyy=0.26;
p_inits=[pxx,pyy,0]';
p_init=p_inits;
rPOSF=position_estimation(rvel,p_init,rPvm);
rposf=rPOSF.pos;
rPposf=rPOSF.Ppm;
% backward
p_init=p_inits;
rPOSB=position_estimation_b(rvel,p_init,rPvm);
rposb=rPOSB.pos;
rPposb=rPOSB.Ppm;
% fusion
[rpos,rK,rPP]=position_fusion(rposf,rposb,rPposf,rPposb);
rpos_1N=rpos;
rP_1n=rPP;


figure
hold on
plot3(rposf(:,1),rposf(:,2),rposf(:,3),'LineWidth',2,'color','g')
plot3(rposb(:,1),rposb(:,2),rposb(:,3),'LineWidth',2,'color','black')
plot3(rpos(:,1),rpos(:,2),rpos(:,3),'LineWidth',2,'color','red')
scatter3(rpos(M,1),rpos(M,2),rpos(M,3),40,'o','filled')
scatter3(rpos(1,1),rpos(1,2),rpos(1,3),120,'filled','pentagram')
xlabel('x (m)','interpreter','latex')
ylabel('y (m)','interpreter','latex')
zlabel('z (m)','interpreter','latex')
view(-100,-8)
legend('posf','posb','pos fusion','S','M','interpreter','latex','Location','southwest')
set(gca,'fontsize',16)
set(gcf,'position',[100 100 750 600])


%% assume that we know the point of M
% position M: (0,0.25,-3.6)
clear rpos_1N_seg rP_1n_segc
%% seg=seg1+seg2
index_seg1=1:M;
index_seg2=M+1:len;
posm=[pxx,pyy,-3.6];
% position estimation： seg1
p_init=p_inits;
rPOSF_seg1=position_estimation(rvel(index_seg1,:),p_init,rPvm(index_seg1,:));
rposf_seg1=rPOSF_seg1.pos;
rPposf_seg1=rPOSF_seg1.Ppm;
% backward
p_init=posm';
rPOSB_seg1=position_estimation_b(rvel(index_seg1,:),p_init,rPvm(index_seg1,:));
rposb_seg1=rPOSB_seg1.pos;
rPposb_seg1=rPOSB_seg1.Ppm;
% fusion
[rpos_seg1,rK_seg1,rPP_seg1]=position_fusion(rposf_seg1,rposb_seg1,rPposf_seg1,rPposb_seg1);
rpos_1N_seg1=rpos_seg1;
rP_1n_seg1=rPP_seg1;

% position estimation： seg2
p_init=posm';
rPOSF_seg2=position_estimation(rvel(index_seg2,:),p_init,rPvm(index_seg2,:));
rposf_seg2=rPOSF_seg2.pos;
rPposf_seg2=rPOSF_seg2.Ppm;
% backward
p_init=p_inits;
rPOSB_seg2=position_estimation_b(rvel(index_seg2,:),p_init,rPvm(index_seg2,:));
rposb_seg2=rPOSB_seg2.pos;
rPposb_seg2=rPOSB_seg2.Ppm;
% fusion
[rpos_seg2,rK_seg2,rPP_seg2]=position_fusion(rposf_seg2,rposb_seg2,rPposf_seg2,rPposb_seg2);
rpos_1N_seg2=rpos_seg2;
rP_1n_seg2=rPP_seg2;

% piece together
rpos_1N_seg=zeros(len,3);
rP_1n_seg=zeros(len,1);
rpos_1N_seg(index_seg1,:)=rpos_1N_seg1;
rpos_1N_seg(index_seg2,:)=rpos_1N_seg2;
rP_1n_seg(index_seg1,:)=rP_1n_seg1;
rP_1n_seg(index_seg2,:)=rP_1n_seg2;

%%
figure
hold on
plot3(rposf(:,1),rposf(:,2),rposf(:,3),'LineWidth',2,'color','g')
plot3(rpos_1N(:,1),rpos_1N(:,2),rpos_1N(:,3),'LineWidth',2,'color','blue')
plot3(rpos_1N_seg(:,1),rpos_1N_seg(:,2),rpos_1N_seg(:,3),'LineWidth',2,'color','red')
scatter3(rpos_1N_seg(M,1),rpos_1N_seg(M,2),rpos_1N_seg(M,3),100,'o','filled')
scatter3(rpos_1N_seg(1,1),rpos_1N_seg(1,2),rpos_1N_seg(1,3),200,'filled','pentagram')
xlabel('x (m)','interpreter','latex')
ylabel('y (m)','interpreter','latex')
zlabel('z (m)','interpreter','latex')
view(-100,-8)
legend('posf','posb','pos fusion','S','M','interpreter','latex','Location','southwest')
set(gca,'fontsize',16)
set(gcf,'position',[100 100 750 600])




%% store
lQuat_ceks_s=rQuat_ceks;
leuler_ceks_s=reuler_ceks;
lposf_s=rposf;
lpos_1N_seg_s=rpos_1N_seg;

%% 
euler_rot=[13,0,0]/180*pi; % init
q_rot= eul2quat(euler_rot,'ZXY'); % b_q initilization
q_rot=quaternion(q_rot);
lQuat_ceks_snew=q_rot.*lQuat_ceks_s.*conj(q_rot);
lpos_1N_seg_sq=quaternion([zeros(length(lpos_1N_seg_s),1),lpos_1N_seg_s]);
lpos_1N_s_new=q_rot.*lpos_1N_seg_sq.*conj(q_rot);
lpos_1N_s_new=compact(lpos_1N_s_new);
lpos_1N_s_new=lpos_1N_s_new(:,2:4);

figure
hold on
plot3(rpos_1N_seg_s(1:end,1),rpos_1N_seg_s(1:end,2),rpos_1N_seg_s(1:end,3),'LineWidth',2,'color','blue')
plot3(lpos_1N_s_new(1:end,1),lpos_1N_s_new(1:end,2),lpos_1N_s_new(1:end,3),'LineWidth',2,'color','red')
scatter3(rpos_1N_seg_s(M,1),rpos_1N_seg_s(M,2),rpos_1N_seg_s(M,3),100,'o','filled')
scatter3(rpos_1N_seg_s(1,1),rpos_1N_seg_s(1,2),rpos_1N_seg_s(1,3),200,'filled','pentagram')
scatter3(lpos_1N_s_new(M,1),lpos_1N_s_new(M,2),lpos_1N_s_new(M,3),100,'o','filled')
scatter3(lpos_1N_s_new(1,1),lpos_1N_s_new(1,2),lpos_1N_s_new(1,3),200,'filled','pentagram')

xlabel('x (m)','interpreter','latex')
ylabel('y (m)','interpreter','latex')
zlabel('z (m)','interpreter','latex')
view(-100,-8)
legend('rpos fusion','lpos fusion','S','M','interpreter','latex','Location','southwest')
set(gca,'fontsize',16)
set(gcf,'position',[100 100 750 600])


%% animation
r_index = IMU.right.index1;
l_index = IMU.left.index2;

rQuat_ceks_ss=compact(rQuat_ceks_s(r_index));
lQuat_ceks_ss=compact(lQuat_ceks_snew(l_index));

rpos_ss=rpos_1N_seg_s(r_index,:);
lpos_ss=lpos_1N_s_new(l_index,:);

rquat2Matrix=quatern2rotMat(rQuat_ceks_ss);
lquat2Matrix=quatern2rotMat(lQuat_ceks_ss);

% animation
fs=100;
samplePeriod=1/fs;
% Create 6 DOF animation
SamplePlotFreq = 8;
Spin = 120;
% 'Trail', 'All'
SixDofAnimationTwin(rpos_ss, rquat2Matrix, lpos_ss,lquat2Matrix,...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
                'Position', [9 39 900 700], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
                'CreateAVI', true, 'AVIfileName','3','AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));
view(41.5,21)

%single foot

% animation
fs=100;
samplePeriod=1/fs;
% Create 6 DOF animation
SamplePlotFreq = 8;
Spin = 120;
SixDofAnimation(rpos_ss, rquat2Matrix,...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
                'Position', [9 39 900 700], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
                'CreateAVI', true, 'AVIfileName','4','AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));
view(41.5,21)

end