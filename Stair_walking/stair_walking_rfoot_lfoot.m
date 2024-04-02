function stair_walking_rfoot_lfoot()
% two feet trajectory estimation of square walking: yes
close all
clear all
addpath(genpath('../Orientation'));
addpath(genpath('./data'));
load('imu_STAIR_1.mat')
%% load right foot imu data
imu=IMU.right;
% obtain the orientation
fs=imu.fs;
Accelerometer=-imu.Acceleration;
Gyroscope=imu.Gyroscope;
Magnetic=imu.Magnetic*45;
len=length(Accelerometer);
for i=1:len
    Mag_norm(i)=norm(Magnetic(i,:));
    Acc_norm(i)=norm(Accelerometer(i,:));
end
Acc_free=imu.FreeAcc;
%% right foot orientation
%% CEKS
sigma_acc_init=2.3;
sigma_mag_init=1.7*0.5;
sigma_acc=sigma_acc_init;
sigma_mag=sigma_mag_init;
rt=0:1/fs:1/fs*(len-1);
stdGyro = 0.001*5;                % (rad/s)
stdAcc = 0.0981;           % (g)
stdMag  = 0.02;          % (a.u.)
%reset_index= [];
[cekf,q1] = SAB_New_MKMC(imu.Acceleration, imu.Gyroscope, imu.Magnetic, rt, stdAcc, stdGyro, stdMag, sigma_acc,sigma_mag);
for i=1:length(q1)
    Quat_cekf(i)=quaternion(q1(i,4),q1(i,1),q1(i,2),q1(i,3));
end
euler_cekf=eulerd(Quat_cekf,'ZXY','frame');
%% smoother
ekfb=SAB_New_MKMCS(cekf,imu.Acceleration,imu.Magnetic);
q1=ekfb.stateb';
Quat_ceks=Quat_cekf;
for i=1:length(q1)
    Quat_ceks(i)=quaternion(q1(i,4),q1(i,1),q1(i,2),q1(i,3));
end
euler_ceks=eulerd(Quat_ceks,'ZXY','frame');
%% right free acceleration
g=[0,0,9.81]; 
acc=Accelerometer;
acc_q=quaternion([zeros(length(acc),1),acc]);
% ceks
g_se=Quat_ceks'; % 
%acc_es=g_se.*acc_q.*conj(g_se); % obtain the acc readings in the inertial frame
acc_es=conj(g_se).*acc_q.*(g_se); % obtain the acc readings in the inertial frame
acc_esv = compact(acc_es);
acc_es=acc_esv(:,2:4);
acc_free=acc_es-g; % free acceleration in the initial frame
freeAcc_ceks=acc_free;
%% segmentation of right foot
clear pos pos_1N P_1n
mydata=[rt',Accelerometer,Gyroscope,Magnetic];
[P,M]=gyroscope_norm(mydata,freeAcc_ceks,1:length(Accelerometer),fs); % the moveing interval except the first row
len=length(rt);
vel=zeros(len,3);
Pvm=zeros(len,1);
%% trajectory estimation: loop
[seg,col]=size(P);
for i=1:seg-1
    index=P(i,2):P(i+1,1);
    acc=freeAcc_ceks(index,:);
    v_init=zeros(3,1);
    % forward velocity estimation
    VEL=velocity_estimation(acc,v_init,4*stdAcc,index);
    % forward position estimation
    % store the vel and Pv
    vel(index,:)=VEL.vel;
    Pvm(index,:)=VEL.Pv; % velocity with covariance
end
% position estimation
p_init=zeros(3,1);
POSF=position_estimation(vel,p_init,Pvm);
posf=POSF.pos;
Pposf=POSF.Ppm;
% backward
POSB=position_estimation_b(vel,p_init,Pvm);
posb=POSB.pos;
Pposb=POSB.Ppm;
% fusion
[pos,K,PP]=position_fusion(posf,posb,Pposf,Pposb);
pos_1N=pos;
P_1n=PP;

%% store： right foot
rindex=IMU.right.index1;
rMag_norm=Mag_norm(rindex);
rQuat_ceks=Quat_ceks(rindex); %
reuler_ceks=euler_ceks(rindex,:);
rposf=posf(rindex,:); rPposf=Pposf(rindex,:);
rpos=pos_1N(rindex,:); rP_1n=P_1n(rindex,:);



figure
x1=subplot(3,1,1);
hold on
plot(reuler_ceks(:,1),'linewidth',0.6)
plot(imu.EulerNew(rindex,1),'linewidth',0.6)
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
plot3(rpos(:,1),rpos(:,2),rpos(:,3),'LineWidth',2,'color','blue')
scatter3(rpos(1,1),rpos(1,2),rpos(1,3),200,'filled','pentagram')
xlabel('x (m)','interpreter','latex')
ylabel('y (m)','interpreter','latex')
zlabel('z (m)','interpreter','latex')
view(-100,-8)
legend('posf','posb','pos fusion','S','M','interpreter','latex','Location','southwest')
set(gca,'fontsize',16)
set(gcf,'position',[100 100 750 600])

% 
% figure
% hold on
% x1=subplot(2,1,1);
% plot(Mag_norm,'linewidth',2,'color','red')
% ylabel('Mag ($\mu T$)','interpreter','latex')
% legend('Mag norm')
% xticks([])
% ylim([10 100])
% set(gca,'fontsize',18)
% x2=subplot(2,1,2);
% plot(Acc_norm,'linewidth',2,'color','red')
% legend('Acc norm')
% xlabel('time (s)','Interpreter','latex')
% ylabel('Acc ($m/s^2$)','interpreter','latex')
% set(gca,'fontsize',18)
% linkaxes([x1,x2],'x')

%% load left foot imu data
imu=IMU.left;
% obtain the orientation
fs=imu.fs;
Accelerometer=-imu.Acceleration;
Gyroscope=imu.Gyroscope;
Magnetic=imu.Magnetic*45;
len=length(Accelerometer);
for i=1:len
    Mag_norm(i)=norm(Magnetic(i,:));
    Acc_norm(i)=norm(Accelerometer(i,:));
end
Acc_free=imu.FreeAcc;
%% right foot orientation
%% CEKS
sigma_acc_init=2.3;
sigma_mag_init=1.7*0.1;
sigma_acc=sigma_acc_init;
sigma_mag=sigma_mag_init;
rt=0:1/fs:1/fs*(len-1);
stdGyro = 0.001*5;                % (rad/s)
stdAcc = 0.0981;           % (g)
stdMag  = 0.02;          % (a.u.)
%reset_index= [];
[cekf,q1] = SAB_New_MKMC(imu.Acceleration, imu.Gyroscope, imu.Magnetic, rt, stdAcc, stdGyro, stdMag, sigma_acc,sigma_mag);
clear Quat_cekf
for i=1:length(q1)
    Quat_cekf(i)=quaternion(q1(i,4),q1(i,1),q1(i,2),q1(i,3));
end
euler_cekf=eulerd(Quat_cekf,'ZXY','frame');
%% smoother
ekfb=SAB_New_MKMCS(cekf,imu.Acceleration,imu.Magnetic);
q1=ekfb.stateb';
Quat_ceks=Quat_cekf;
for i=1:length(q1)
    Quat_ceks(i)=quaternion(q1(i,4),q1(i,1),q1(i,2),q1(i,3));
end
euler_ceks=eulerd(Quat_ceks,'ZXY','frame');
%% right free acceleration
g=[0,0,9.81]; 
acc=Accelerometer;
acc_q=quaternion([zeros(length(acc),1),acc]);
% ceks
g_se=Quat_ceks'; % 
%acc_es=g_se.*acc_q.*conj(g_se); % obtain the acc readings in the inertial frame
acc_es=conj(g_se).*acc_q.*(g_se); % obtain the acc readings in the inertial frame
acc_esv = compact(acc_es);
acc_es=acc_esv(:,2:4);
acc_free=acc_es-g; % free acceleration in the initial frame
freeAcc_ceks=acc_free;
%% segmentation of right foot
clear pos pos_1N P_1n
mydata=[rt',Accelerometer,Gyroscope,Magnetic];
[P,M]=gyroscope_norm(mydata,freeAcc_ceks,1:length(Accelerometer),fs); % the moveing interval except the first row
len=length(rt);
vel=zeros(len,3);
Pvm=zeros(len,1);
%% trajectory estimation: loop
[seg,col]=size(P);
for i=1:seg-1
    index=P(i,2):P(i+1,1);
    acc=freeAcc_ceks(index,:);
    v_init=zeros(3,1);
    % forward velocity estimation
    VEL=velocity_estimation(acc,v_init,4*stdAcc,index);
    % forward position estimation
    % store the vel and Pv
    vel(index,:)=VEL.vel;
    Pvm(index,:)=VEL.Pv; % velocity with covariance
end
% position estimation
p_init=zeros(3,1);
POSF=position_estimation(vel,p_init,Pvm);
posf=POSF.pos;
Pposf=POSF.Ppm;
% backward
POSB=position_estimation_b(vel,p_init,Pvm);
posb=POSB.pos;
Pposb=POSB.Ppm;
% fusion
[pos,K,PP]=position_fusion(posf,posb,Pposf,Pposb);
pos_1N=pos;
P_1n=PP;

%% store： left foot
lindex=IMU.left.index2;
lMag_norm=Mag_norm(lindex);
lQuat_ceks=Quat_ceks(lindex); %
leuler_ceks=euler_ceks(lindex,:);
lposf=posf(lindex,:); lPposf=Pposf(lindex,:);
lpos=pos_1N(lindex,:); lP_1n=P_1n(lindex,:);


figure
x1=subplot(3,1,1);
hold on
plot(reuler_ceks(:,1),'linewidth',0.6)
plot(leuler_ceks(:,1),'linewidth',0.6)
plot(imu.EulerNew(lindex,1),'linewidth',0.6)
legend('REKS','LEKS','Xsens')
x2=subplot(3,1,2);
hold on
plot(leuler_ceks(:,2),'linewidth',0.6)
x3=subplot(3,1,3);
hold on
plot(leuler_ceks(:,3),'linewidth',0.6)
linkaxes([x1,x2,x3],'x')

figure
hold on
plot3(lposf(:,1),lposf(:,2),lposf(:,3),'LineWidth',2,'color','g')
plot3(lpos(:,1),lpos(:,2),lpos(:,3),'LineWidth',2,'color','blue')
plot3(rpos(:,1),rpos(:,2),rpos(:,3),'LineWidth',2,'color','red')
scatter3(lpos(1,1),lpos(1,2),lpos(1,3),200,'filled','pentagram')
xlabel('x (m)','interpreter','latex')
ylabel('y (m)','interpreter','latex')
zlabel('z (m)','interpreter','latex')
view(-100,-8)
legend('posf','posb','r pos','S','interpreter','latex','Location','southwest')
set(gca,'fontsize',16)
set(gcf,'position',[100 100 750 600])


figure
hold on
x1=subplot(2,1,1);
hold on
plot(rMag_norm,'linewidth',2,'color','red')
ylabel('Mag ($\mu T$)','interpreter','latex')
legend('r Mag norm')
xticks([])
ylim([10 100])
set(gca,'fontsize',18)
x2=subplot(2,1,2);
plot(lMag_norm,'linewidth',2,'color','blue')
legend('l Mag norm')
xlabel('time (s)','Interpreter','latex')
ylabel('Mag ($\mu T$)','interpreter','latex')
set(gca,'fontsize',18)
linkaxes([x1,x2],'x')




%% rotate both right foot and left foot
euler_rot=[6,0,0]/180*pi; % init
q_rot= eul2quat(euler_rot,'ZXY'); % b_q initilization
q_rot=quaternion(q_rot);
rQuat_ceks_b=q_rot.*rQuat_ceks.*conj(q_rot); % for animation: building frame
rpos_q=quaternion([zeros(length(rpos),1),rpos]);
rpos_q=q_rot.*rpos_q.*conj(q_rot);
rpos_b=compact(rpos_q);
rpos_b=rpos_b(:,2:4); % building frame

euler_rot=[24,0,0]/180*pi; % init
q_rot= eul2quat(euler_rot,'ZXY'); % b_q initilization
q_rot=quaternion(q_rot);
lQuat_ceks_b=q_rot.*lQuat_ceks.*conj(q_rot); % for animation
lpos_q=quaternion([zeros(length(lpos),1),lpos]);
lpos_q=q_rot.*lpos_q.*conj(q_rot);
lpos_b=compact(lpos_q);
lpos_b=lpos_b(:,2:4);

% shift the left foot
center=[-0.2,0,0];
lpos_b=lpos_b+center;

figure
hold on
plot3(rpos_b(:,1),rpos_b(:,2),rpos_b(:,3),'LineWidth',2,'color','blue')
plot3(lpos_b(:,1),lpos_b(:,2),lpos_b(:,3),'LineWidth',2,'color','red')
scatter3(lpos_b(1,1),lpos_b(1,2),lpos_b(1,3),200,'filled','pentagram')
xlabel('x (m)','interpreter','latex')
ylabel('y (m)','interpreter','latex')
zlabel('z (m)','interpreter','latex')
view(-100,-8)
legend('r pos','l pos','interpreter','latex','Location','southwest')
set(gca,'fontsize',16)
set(gcf,'position',[100 100 750 600])


%% animation


rpos_ss=rpos_b;
lpos_ss=lpos_b;

rQuat_ceks_bc=compact(rQuat_ceks_b);
lQuat_ceks_bc=compact(lQuat_ceks_b);
rquat2Matrix=quatern2rotMat(rQuat_ceks_bc);
lquat2Matrix=quatern2rotMat(lQuat_ceks_bc);

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
                'CreateAVI', false, 'AVIfileName','3','AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));
view(41.5,21)

%single foot

% % animation
% fs=100;
% samplePeriod=1/fs;
% % Create 6 DOF animation
% SamplePlotFreq = 8;
% Spin = 120;
% SixDofAnimation(rpos_ss, rquat2Matrix,...
%                 'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
%                 'Position', [9 39 900 700], ...
%                 'AxisLength', 0.1, 'ShowArrowHead', false, ...
%                 'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
%                 'CreateAVI', true, 'AVIfileName','4','AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));
% view(41.5,21)

end