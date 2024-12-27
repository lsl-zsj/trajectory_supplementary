function stair_walking_pilot_s3_intersection_shift()
% two feet trajectory estimation of square walking: yes
close all
clear all
addpath(genpath('D:\Work\Task\IMU_Code\IMU-Matlab'));
addpath(genpath('D:\AfterPhD\research\FootTrajectoryEstimation\data_Xsens\SynchronizationAndVisualization\data'));
load('IMU_Shift.mat')
%%
rIMU=IMU;

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
%% orientation estimation using MKMC and EKS
% MagSth=45;
% sigma_1=1.6188;
% sigma_2=0.4234;
% sigma1=2*sigma_1*sigma_1;
% sigma2=2*sigma_2*sigma_2;
% xigma_x=[10^8 10^8 10^8 10^8 10^8 10^8 sigma1 sigma1 sigma1 sigma2 sigma2 sigma2]; 
% xigma_y=[10^8 10^8 10^8 10^8 10^8 10^8];
% mkmc_ahrs=orientation_estimation_ahrs_mkmc_fun_xsens(Accelerometer,Gyroscope,Magnetic,fs,xigma_x,xigma_y,MagSth);
% Quat_mkmc=mkmc_ahrs.Quat;
% euler_mkmc=eulerd(Quat_mkmc,'ZXY','frame');
%% right foot
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
% position estimation (global)
p_init=zeros(3,1);
rPOSF=position_estimation(rvel,p_init,rPvm);
rposf=rPOSF.pos;
rPposf=rPOSF.Ppm;
% backward
p_init=rposf(end,:);
p_init(3)=0;
rPOSB=position_estimation_b(rvel,p_init,rPvm);
rposb=rPOSB.pos;
rPposb=rPOSB.Ppm;
% fusion
[rpos,rK,rPP]=position_fusion(rposf,rposb,rPposf,rPposb);
rpos_1N=rpos;
rP_1n=rPP;


% 
figure
hold on
plot3(rposf(:,1),rposf(:,2),rposf(:,3),'LineWidth',2,'color','g')
plot3(rposb(:,1),rposb(:,2),rposb(:,3),'LineWidth',2,'color','black')
plot3(rpos(:,1),rpos(:,2),rpos(:,3),'LineWidth',2,'color','m')
xlabel('x (m)','interpreter','latex')
ylabel('y (m)','interpreter','latex')
zlabel('z (m)','interpreter','latex')
view(-100,-8)
legend('OMC','CEKSF','CEKSB','CEKS Fusion','interpreter','latex','Location','southwest')
legend('CEKSF','CEKSB','CEKS Fusion','interpreter','latex','Location','southwest')
set(gca,'fontsize',16)
set(gcf,'position',[100 100 750 600])



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

%%
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


end