function walking_square()

% foot trajectory estimation of stair walking 1：yes yes
close all
clear all
addpath(genpath('../Data'));
addpath(genpath('../Orientation'));
load('SQUARE_S2.mat')
load('IMU_SQUARE_S2.mat')
% obtain the orientation
fs=IMU.fs;
sample_freq=fs;
Accelerometer=-IMU.Acceleration;
Gyroscope=IMU.Gyroscope;
Magnetic=IMU.Magnetic*45;
len=length(Accelerometer);
for i=1:len
    Acc_norm(i)=norm(Accelerometer(i,:));
    Mag_norm(i)=norm(Magnetic(i,:));
end
%% orientation estimation using ESKF, GD, DOE, MKMC, and EKF
%% ESKF
MagSth=45;
ahrs=orientation_estimation_ahrs_fun_xsens(Accelerometer,Gyroscope,Magnetic,fs,MagSth);
Quat_eskf=ahrs.Quat;
euler_eskf=eulerd(Quat_eskf,'ZXY','frame');
%% MadgwickAHRS
AHRS = MadgwickAHRS('SamplePeriod', 1/fs, 'Beta', 0.05);
time=0:1/fs:1/fs*(len-1);
quat = zeros(length(time), 4);
Err = zeros(length(time), 6);
for t = 1:length(time)
    AHRS.Update(Gyroscope(t,:), Accelerometer(t,:), Magnetic(t,:));	% gyroscope units must be radians
    quat(t, :) = AHRS.Quaternion;
    Err(t,:)=AHRS.Err;
end
% Plot algorithm output as Euler angles
Quat_gd=Quat_eskf;
for i=1:length(quat)
Quat_gd(i)=quaternion(quat(i,1),quat(i,2),quat(i,3),quat(i,4));
end
euler_gd=eulerd(Quat_gd,'ZXY','frame');
%% DOE
tauAcc= 5;
tauMag= 10;
zeta= 0.1;
accRating= 0;
out =EMDI(Accelerometer,Gyroscope,Magnetic,sample_freq, tauAcc, tauMag, zeta, accRating);
quat_doe=out.q;
Quat_doe=Quat_gd;
for i=1:length(quat_doe)
Quat_doe(i)=quaternion(quat_doe(i,:));
end
euler_doe=eulerd(Quat_doe,'ZXY','frame');
%% MKMC
MagSth=45;
sigma_1=1.6188;
sigma_2=0.4234;
sigma1=2*sigma_1*sigma_1;
sigma2=2*sigma_2*sigma_2;
xigma_x=[10^8 10^8 10^8 10^8 10^8 10^8 sigma1 sigma1 sigma1 sigma2 sigma2 sigma2]; 
xigma_y=[10^8 10^8 10^8 10^8 10^8 10^8];
mkmc_ahrs=orientation_estimation_ahrs_mkmc_fun_xsens(Accelerometer,Gyroscope,Magnetic,fs,xigma_x,xigma_y,MagSth);
Quat_mkmc=mkmc_ahrs.Quat;
euler_mkmc=eulerd(Quat_mkmc,'ZXY','frame');
%% EKF
sigma_acc_init=1000;
sigma_mag_init=1000;
sigma_acc=sigma_acc_init;
sigma_mag=sigma_mag_init;
t=0:1/fs:1/fs*(len-1);
stdGyro = 0.001*5;                % (rad/s)
stdAcc = 0.0981;           % (g)
stdMag  = 0.02;          % (a.u.)
[ekf,q1] = SAB_New_MKMC(IMU.Acceleration, IMU.Gyroscope, IMU.Magnetic, t, stdAcc, stdGyro, stdMag, sigma_acc,sigma_mag);
for i=1:length(q1)
    Quat_ekf(i)=quaternion(q1(i,4),q1(i,1),q1(i,2),q1(i,3));
end
euler_ekf=eulerd(Quat_ekf,'ZXY','frame');
%% EK smoother
ekfb=SAB_New_MKMCS(ekf,IMU.Acceleration,IMU.Magnetic);
q1=ekfb.stateb';
Quat_eks=Quat_mkmc;
for i=1:length(q1)
    Quat_eks(i)=quaternion(q1(i,4),q1(i,1),q1(i,2),q1(i,3));
end
euler_eks=eulerd(Quat_eks,'ZXY','frame');
%% CEKF
sigma_acc_init=2.3;
sigma_mag_init=1.7;
sigma_acc=sigma_acc_init;
sigma_mag=sigma_mag_init;
t=0:1/fs:1/fs*(len-1);
stdGyro = 0.001*5;                % (rad/s)
stdAcc = 0.0981;           % (g)
stdMag  = 0.02;          % (a.u.)
[cekf,q1] = SAB_New_MKMC(IMU.Acceleration, IMU.Gyroscope, IMU.Magnetic, t, stdAcc, stdGyro, stdMag, sigma_acc,sigma_mag);
for i=1:length(q1)
    Quat_cekf(i)=quaternion(q1(i,4),q1(i,1),q1(i,2),q1(i,3));
end
euler_cekf=eulerd(Quat_cekf,'ZXY','frame');
%% CEK smoother
ekfb=SAB_New_MKMCS(cekf,IMU.Acceleration,IMU.Magnetic);
q1=ekfb.stateb';
Quat_ceks=Quat_mkmc;
for i=1:length(q1)
    Quat_ceks(i)=quaternion(q1(i,4),q1(i,1),q1(i,2),q1(i,3));
end
euler_ceks=eulerd(Quat_ceks,'ZXY','frame');
%% compare the orientation error of different method : rough comparison
Euler=imuMC.Euler;
euler_imu=euler_ceks;
t=0:1/fs:1/fs*(length(Accelerometer)-1);
% findpeak method
cur1=Euler(:,2);
cur2=-euler_imu(:,2);
[pks1,locs1] = findpeaks(cur1,'MinPeakHeight',10,'MinPeakDistance',100);
[pks2,locs2] = findpeaks(cur2,'MinPeakHeight',10,'MinPeakDistance',100);
% 
%t_offset=t(locs2(1))-imuMC.t(locs1(1));
t_offset=t(locs2(1:3))-(imuMC.t(locs1(1:3)))'; % average the time gap of three peaks
t_offset=mean(t_offset);
imuMC.t=imuMC.t+t_offset;   % time alignment: this is important
% 
figure
x1=subplot(3,1,1);
hold on
plot(imuMC.t,Euler(:,1)+102.5,'linewidth',0.6)
plot(t,-euler_imu(:,1),'linewidth',0.6)
plot(t,-euler_mkmc(:,1),'linewidth',0.6)
plot(t,-euler_gd(:,1),'linewidth',0.6)
legend('Vicon','EKS','MKMC','GD')
x2=subplot(3,1,2);
hold on
plot(imuMC.t,Euler(:,2)-0.8,'linewidth',0.6)
plot(t,-euler_imu(:,2),'linewidth',0.6)
plot(t,-euler_mkmc(:,2),'linewidth',0.6)
plot(t,-euler_gd(:,2),'linewidth',0.6)
plot(imuMC.t(locs1),pks1-0.8,'o')
plot(t(locs2),pks2,'<')
x3=subplot(3,1,3);
hold on
plot(imuMC.t,Euler(:,3)-92,'linewidth',0.6)
plot(t,euler_imu(:,3),'linewidth',0.6)
plot(t,euler_mkmc(:,3),'linewidth',0.6)
plot(t,euler_gd(:,3),'linewidth',0.6)
linkaxes([x1,x2,x3],'x')
%% plot the sensor norm readings

t_ss=3.2;
t_ee=24.4;

figure
hold on
x1=subplot(2,1,1);
plot(t-t_ss,Mag_norm,'linewidth',2,'color','red')
ylabel('Mag ($\mu T$)','interpreter','latex')
legend('Mag norm')
xticks([])
ylim([32 50])
set(gca,'fontsize',18)
x2=subplot(2,1,2);
plot(t-t_ss,Acc_norm,'linewidth',2,'color','red')
legend('Acc norm')
xlabel('time (s)','Interpreter','latex')
ylabel('Acc ($m/s^2$)','interpreter','latex')
set(gca,'fontsize',18)
linkaxes([x1,x2],'x')
xlim([0,t_ee-t_ss])

%% ZUPT
% stance and swing detection
mydata=[t',Accelerometer,Gyroscope,Magnetic];
P=gyroscope_norm(mydata,Accelerometer,1:len,fs); % the moveing interval except the first row
% trajectory obtainment
t_s=t(P(1,2)-200); % left shift 0.5s
t_e=t(P(end,1)+200); % right shift 0.5s
t_s=round(t_s,1);
t_e=round(t_e,1);
clear index_tracker index_imu
index_tracker=find(imuMC.t>=t_s&imuMC.t<=t_e); % index for tracker
index_tracker=index_tracker';
index_imu=find(t>=t_s&t<=t_e); % index for imu
% alignment
if(length(index_imu)>=4*length(index_tracker)) % unify the sampling time
    index_imu(4*length(index_tracker)+1:end)=[];
else
    index_tracker(end)=[];
    index_imu(4*length(index_tracker)+1:end)=[];
end
q_mc_q=imuMC.quat(index_tracker,:); %
q_mc_q=quaternion(q_mc_q);%
q_imu_q=Quat_ceks(index_imu,:);% using the ceks as the baseline
q_imu_q=q_imu_q(1:4:end,:); % sampling alginment 
%% alignment optimization
euler_kf=[-108,0,-180]/180*pi; % init
q_kf1 = eul2quat(euler_kf,'ZXY'); % b_q initilization
euler_kf=[0,0,-90]/180*pi;
q_kf2 = eul2quat(euler_kf,'ZXY'); % a_q initilization
q_kf_q1=quaternion(q_kf1);
q_kf_q2=quaternion(q_kf2);
q_imu_left=compact(q_kf_q1); % b_q initilization
q_mc_right=compact(conj(q_kf_q2));   % a_q initilization
% q_{ol}^{og}= q_{ig}^{ol} * q_{il}^{ig} * q_{ol}^{il}  = b_q.*q_imu_ .* conj(a_q)
global q_mc_ q_imu_
q_mc_=q_mc_q;
q_imu_=q_imu_q;
% q_mc_(i,:)=b_q.*q_imu_(i,:)*conj(a_q);
x0(1:4)=q_mc_right; % a_q  =  q_kf_q1
x0(5:8)=q_imu_left; % b_q = q_kf_q2
fun=@nonlinear_func;
mycon=@constrains;
options = optimoptions('fmincon','Display','iter');
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,mycon,options);
display(x);
a_q=quaternion(x(1:4)); % obtain the result
a_q=normalize(a_q);
b_q=quaternion(x(5:8)); % obtain the result
b_q=normalize(b_q);
tar=compact(q_mc_);
act=compact(b_q.*q_imu_.*conj(a_q));
% figure
% plot(tar,'linewidth',0.5)
% hold on
% plot(-act,'linewidth',0.5)
% legend('OMC quaternion','IMU quaternion')

%% orientation error comparison
q_imu_eks=Quat_eks(index_imu,:);% smoother
q_imu_eks=q_imu_eks(1:4:end,:); % sampling alginment 
q_imu_eks_mc=-b_q.*q_imu_eks.*conj(a_q);

q_imu_ceks=Quat_ceks(index_imu,:);% smoother
q_imu_ceks=q_imu_ceks(1:4:end,:); % sampling alginment 
q_imu_ceks_mc=-b_q.*q_imu_ceks.*conj(a_q);

q_imu_eskf=Quat_eskf(index_imu,:);% eskf
q_imu_eskf=q_imu_eskf(1:4:end,:); % sampling alginment 
q_imu_eskf_mc=-b_q.*q_imu_eskf.*conj(a_q);

q_imu_gd=Quat_gd(index_imu,:);% gd
q_imu_gd=q_imu_gd(1:4:end,:); % sampling alginment 
q_imu_gd_mc=-b_q.*q_imu_gd.*conj(a_q);

q_imu_doe=Quat_doe(index_imu,:);% doe
q_imu_doe=q_imu_doe(1:4:end,:); % sampling alginment
q_imu_doe_mc=-b_q.*q_imu_doe.*conj(a_q);

q_imu_mkmc=Quat_mkmc(index_imu,:);% mkmc
q_imu_mkmc=q_imu_mkmc(1:4:end,:); % sampling alginment 
q_imu_mkmc_mc=-b_q.*q_imu_mkmc.*conj(a_q);
%
mc=eulerd(q_mc_q,'ZXY','frame');
eks=eulerd(q_imu_eks_mc,'ZXY','frame');
ceks=eulerd(q_imu_ceks_mc,'ZXY','frame');
eskf=eulerd(q_imu_eskf_mc,'ZXY','frame');
gd=eulerd(q_imu_gd_mc,'ZXY','frame');
doe=eulerd(q_imu_doe_mc,'ZXY','frame');
mkmc=eulerd(q_imu_mkmc_mc,'ZXY','frame');
%
err_eks=mc-eks;
err_ceks=mc-ceks;
err_eskf=mc-eskf;
err_gd=mc-gd;
err_doe=mc-doe;
err_mkmc=mc-mkmc;
% yaw error correction
lenEuler=length(err_eks);
for i=1:lenEuler
    if(err_eks(i,1)>100)
    err_eks(i,1)=err_eks(i,1)-360;
    elseif(err_eks(i,1)<-100)
    err_eks(i,1)=err_eks(i,1)+360;
    end
    if(err_ceks(i,1)>100)
    err_ceks(i,1)=err_ceks(i,1)-360;
    elseif(err_ceks(i,1)<-100)
    err_ceks(i,1)=err_ceks(i,1)+360;
    end
    if(err_eskf(i,1)>100)
    err_eskf(i,1)=err_eskf(i,1)-360;
    elseif(err_eskf(i,1)<-100)
    err_eskf(i,1)=err_eskf(i,1)+360;
    end
    if(err_gd(i,1)>100)
    err_gd(i,1)=err_gd(i,1)-360;
    elseif(err_gd(i,1)<-100)
    err_gd(i,1)=err_gd(i,1)+360;
    end
    if(err_doe(i,1)>100)
    err_doe(i,1)=err_doe(i,1)-360;
    elseif(err_doe(i,1)<-100)
    err_doe(i,1)=err_doe(i,1)+360;
    end
end

%
error.err_eks_rms=rms(err_eks);
error.err_ceks_rms=rms(err_ceks);
error.err_eskf_rms=rms(err_eskf);
error.err_gd_rms=rms(err_gd);
error.err_doe_rms=rms(err_doe);
error.err_mkmc_rms=rms(err_mkmc);

figure
t_eul=0:1/100:(length(err_eks)-1)*1/100;
x1=subplot(3,1,1);
hold on
plot(t_eul,err_eks(:,1),'LineWidth',1,'color','g')
plot(t_eul,err_ceks(:,1),'-','LineWidth',2,'color','black','MarkerSize',10,'MarkerIndices',1:80:length(t_eul))
plot(t_eul,err_eskf(:,1),'LineWidth',1,'color','blue')
plot(t_eul,err_gd(:,1),'LineWidth',1,'color','m')
plot(t_eul,err_doe(:,1),'LineWidth',1,'color',[0.4940 0.1840 0.5560])
%plot(err_mkmc(:,1),'linewidth',0.8)
legend('EKS','MKCERTS','ESKF','GD','DOE','interpreter','latex','Orientation','horizontal')
xticks([])
ylabel('yaw ($\deg$)', 'interpreter','latex')
set(gca,'FontSize',16)
box on
x2=subplot(3,1,2);
hold on
plot(t_eul,err_eks(:,2),'LineWidth',1,'color','g')
plot(t_eul,err_ceks(:,2),'-','LineWidth',2,'color','black','MarkerSize',10,'MarkerIndices',1:80:length(t_eul))
plot(t_eul,err_eskf(:,2),'LineWidth',1,'color','blue')
plot(t_eul,err_gd(:,2),'LineWidth',1,'color','m')
plot(t_eul,err_doe(:,2),'LineWidth',1,'color',[0.4940 0.1840 0.5560])
ylabel('roll ($\deg$)', 'interpreter','latex')
%plot(err_mkmc(:,2),'linewidth',0.8)
xticks([])
set(gca,'FontSize',16)
box on
x3=subplot(3,1,3);
hold on
plot(t_eul,err_eks(:,3),'LineWidth',1,'color','g')
plot(t_eul,err_ceks(:,3),'-','LineWidth',2,'color','black','MarkerSize',10,'MarkerIndices',1:80:length(t_eul))
plot(t_eul,err_eskf(:,3),'LineWidth',1,'color','blue')
plot(t_eul,err_gd(:,3),'LineWidth',1,'color','m')
plot(t_eul,err_doe(:,3),'LineWidth',1,'color',[0.4940 0.1840 0.5560])
%plot(err_mkmc(:,3),'linewidth',0.8)
set(gca,'FontSize',16)
xlabel('time (s)', 'interpreter','latex')
ylabel('pitch ($\deg$)', 'interpreter','latex')
box on
linkaxes([x1,x2,x3],'x')
xlim([0,t_eul(end)])
set(gcf,'position',[100 100 750 600])

%% free acceleration
g=[0,0,9.81]; 
acc=Accelerometer;
acc_q=quaternion([zeros(length(acc),1),acc]);
% eks
g_se=Quat_eks; % 
acc_es=g_se.*acc_q.*conj(g_se); % obtain the acc readings in the inertial frame
acc_esv = compact(acc_es);
acc_es=acc_esv(:,2:4);
acc_free=acc_es-g; % free acceleration in the initial frame
freeAcc_eks=acc_free;
% ceks
g_se=Quat_ceks; % 
acc_es=g_se.*acc_q.*conj(g_se); % obtain the acc readings in the inertial frame
acc_esv = compact(acc_es);
acc_es=acc_esv(:,2:4);
acc_free=acc_es-g; % free acceleration in the initial frame
freeAcc_ceks=acc_free;
% eskf
g_se=Quat_eskf; % 
acc_es=g_se.*acc_q.*conj(g_se); % obtain the acc readings in the inertial frame
acc_esv = compact(acc_es);
acc_es=acc_esv(:,2:4);
acc_free=acc_es-g; % free acceleration in the initial frame
freeAcc_eskf=acc_free;
% gd
g_se=Quat_gd; % 
acc_es=g_se.*acc_q.*conj(g_se); % obtain the acc readings in the inertial frame
acc_esv = compact(acc_es);
acc_es=acc_esv(:,2:4);
acc_free=acc_es-g; % free acceleration in the initial frame
freeAcc_gd=acc_free;
% doe
g_se=Quat_doe; % 
acc_es=g_se.*acc_q.*conj(g_se); % obtain the acc readings in the inertial frame
acc_esv = compact(acc_es);
acc_es=acc_esv(:,2:4);
acc_free=acc_es-g; % free acceleration in the initial frame
freeAcc_doe=acc_free;
% mkmc
g_se=Quat_mkmc; % 
acc_es=g_se.*acc_q.*conj(g_se); % obtain the acc readings in the inertial frame
acc_esv = compact(acc_es);
acc_es=acc_esv(:,2:4);
acc_free=acc_es-g; % free acceleration in the initial frame
freeAcc_mkmc=acc_free;
%% segmentation
mydata=[t',Accelerometer,Gyroscope,Magnetic];
P=gyroscope_norm(mydata,freeAcc_ceks,index_imu,fs); % the moveing interval except the first row
len=length(t);
vel=zeros(len,3);
Pvm=zeros(len,1);
%% trajectory estimation
clear freeAcc
freeAcc{1}=freeAcc_eks;
freeAcc{2}=freeAcc_ceks;
freeAcc{3}=freeAcc_eskf;
freeAcc{4}=freeAcc_gd;
freeAcc{5}=freeAcc_doe;
freeAcc{6}=freeAcc_mkmc;
for item=1:6
[seg,col]=size(P);
for i=1:seg-1
    index=P(i,2):P(i+1,1); % swing index
    acc=freeAcc{item}(index,:);
    v_init=zeros(3,1);
    % zero velocity update estimation
    ZUPT_vel=velocity_estimation(acc,v_init,4*stdAcc,index);
    % store the vel and Pv
    vel(index,:)=ZUPT_vel.vel; % velocity estimate
    Pvm(index,:)=ZUPT_vel.Pv; % velocity covariance
end
% position estimation (navigation frame): forward
p_init=zeros(3,1);
POSF=position_estimation(vel,p_init,Pvm);
posf=POSF.pos;
Pposf=POSF.Ppm;
% position estimation (navigation frame): backward
POSB=position_estimation_b(vel,p_init,Pvm);
posb=POSB.pos;
Pposb=POSB.Ppm;
% position fusion
[pos,K,PP]=position_fusion(posf,posb,Pposf,Pposb);
% store trajectory
POSIMU.pos=pos(index_imu,:); % segmentation
POSIMU.P=PP(index_imu,:); %
POSIMU.pos=POSIMU.pos(1:4:end,:); % align the sampling rate
POSIMU.P=POSIMU.P(1:4:end,:); % align the sampling rate
% store the trajectory in navigation frame
POS_store{item}=POSIMU; % navigation frame

% position in the optical motion capture frame
pos_q=quaternion(-[zeros(length(POSIMU.pos),1),POSIMU.pos]); 
pos_q_opc=b_q.*pos_q.*conj(b_q); % b_q = q_{i}^{o} the orientation of the global omc w.r.t. inertial frame
pos_opc=compact(pos_q_opc);
pos_opc=pos_opc(:,2:4);
% store the trajectory in motion capture frame
POS_store_omc{item}.pos=pos_opc;
POS_store_omc{item}.P=POS_store{item}.P;

% forward position in the optical motion capture frame
posf=posf(index_imu,:); % segmentation
posf=posf(1:4:end,:);   % sample aligment
posf_q=quaternion(-[zeros(length(posf),1),posf]); 
posf_q_opc=b_q.*posf_q.*conj(b_q); % b_q = q_{i}^{o} the orientation of the global omc w.r.t. inertial frame
posf_opc=compact(posf_q_opc);
posf_opc=posf_opc(:,2:4);
% store the trajectory in motion capture frame
POS_store_omc{item}.posf=posf_opc;
Pposf_algin=Pposf;
Pposf_algin=Pposf_algin(index_imu,:);
Pposf_algin=Pposf_algin(1:4:end,:);
POS_store_omc{item}.Pf=Pposf_algin;
end

%% trajectory in the navigation frame
% ground truth
fs_omc=100;
index_omc_seg=find(imuMC.t>=t_s&imuMC.t<=t_e);
t_omc=0:1/fs_omc:(length(index_omc_seg)-1)/fs_omc;
poso=imuMC.Tran(index_omc_seg,:);
pos_omc=poso-poso(1,:);
% convert it to navigation frame
pos_omc_q=-quaternion([zeros(length(pos_omc),1),pos_omc]);
pos_omc_n=conj(b_q).*pos_omc_q.*b_q;
pos_omc_n=compact(pos_omc_n);
pos_omc_nimu=pos_omc_n(:,2:4);
% 
figure
hold on
plot3(pos_omc_nimu(:,1),pos_omc_nimu(:,2),pos_omc_nimu(:,3),'-o','LineWidth',2,...
    'Color','red','MarkerIndices',1:10:length(pos_omc_nimu(:,1)),'LineWidth',2)
hold on
plot3(POS_store{1}.pos(:,1),POS_store{1}.pos(:,2),POS_store{1}.pos(:,3),'LineWidth',2,'color','g')
plot3(POS_store{2}.pos(:,1),POS_store{2}.pos(:,2),POS_store{2}.pos(:,3),'LineWidth',2,'color','black','Marker','+','MarkerIndices',1:10:length(POS_store{2}.pos(:,1)))
plot3(POS_store{3}.pos(:,1),POS_store{3}.pos(:,2),POS_store{3}.pos(:,3),'LineWidth',2,'color','blue')
plot3(POS_store{4}.pos(:,1),POS_store{4}.pos(:,2),POS_store{4}.pos(:,3),'LineWidth',2,'color','m')
plot3(POS_store{5}.pos(:,1),POS_store{5}.pos(:,2),POS_store{5}.pos(:,3),'LineWidth',2,'color',[0.4940 0.1840 0.5560])
%plot3(POS_store{6}.pos(:,1),POS_store{6}.pos(:,2),POS_store{6}.pos(:,3),'LineWidth',2,'color',[0.8500 0.3250 0.0980])
xlabel('x (m)','interpreter','latex')
ylabel('y (m)','interpreter','latex')
zlabel('z (m)','interpreter','latex')
view(-87,15)
legend('OMC','EKS','MKCERTS','ESKF','GD','DOE','interpreter','latex','Location','southwest')
set(gca,'fontsize',16)
set(gcf,'position',[100 100 750 600])

%% we visualize the trajectory in motion capture frame
figure
hold on
plot3(pos_omc(:,1),pos_omc(:,2),pos_omc(:,3),'-o','LineWidth',2,...
    'Color','red','MarkerIndices',1:10:length(pos_omc_nimu(:,1)),'LineWidth',2)
hold on
plot3(POS_store_omc{1}.pos(:,1),POS_store_omc{1}.pos(:,2),POS_store_omc{1}.pos(:,3),'LineWidth',2,'color','g')
plot3(POS_store_omc{2}.pos(:,1),POS_store_omc{2}.pos(:,2),POS_store_omc{2}.pos(:,3),'LineWidth',2,'color','black','Marker','+','MarkerIndices',1:10:length(POS_store{2}.pos(:,1)))
plot3(POS_store_omc{3}.pos(:,1),POS_store_omc{3}.pos(:,2),POS_store_omc{3}.pos(:,3),'LineWidth',2,'color','blue')
plot3(POS_store_omc{4}.pos(:,1),POS_store_omc{4}.pos(:,2),POS_store_omc{4}.pos(:,3),'LineWidth',2,'color','m')
plot3(POS_store_omc{5}.pos(:,1),POS_store_omc{5}.pos(:,2),POS_store_omc{5}.pos(:,3),'LineWidth',2,'color',[0.4940 0.1840 0.5560])
%plot3(POS_store_omc{6}.pos(:,1),POS_store_omc{6}.pos(:,2),POS_store_omc{6}.pos(:,3),'LineWidth',2,'color',[0.8500 0.3250 0.0980])
xlabel('x (m)','interpreter','latex')
ylabel('y (m)','interpreter','latex')
zlabel('z (m)','interpreter','latex')
view(-90,90)
legend('OMC','EKS','MKCERTS','ESKF','GD','DOE','interpreter','latex','Location','northwest')
set(gca,'fontsize',16)
set(gcf,'position',[100 100 750 600])
box on
%% obtain the error and error rate
for item=1:6
    POS_store_omc{item}.pos_err=POS_store_omc{item}.pos-pos_omc; % closed loop error
    POS_store_omc{item}.pos_errf=POS_store_omc{item}.posf-pos_omc; % open loop error
    nlen=length(POS_store_omc{item}.pos);
    delta_pos_norm=zeros(nlen,1);
    delta_pos_norm_accm=zeros(nlen,1);
    POS_store_omc{item}.pos_err_norm=zeros(nlen,1);
    POS_store_omc{item}.pos_errf_norm=zeros(nlen,1);
    for i=1:nlen
        POS_store_omc{item}.pos_err_norm(i)=norm(POS_store_omc{item}.pos_err(i,:));
        POS_store_omc{item}.pos_errf_norm(i)=norm(POS_store_omc{item}.pos_errf(i,:));
        if(i>1)
           delta_pos_norm(i)=norm(pos_omc(i,:)-pos_omc(i-1,:)); % delta trajectory length 
           delta_pos_norm_accm(i)=delta_pos_norm_accm(i-1)+delta_pos_norm(i);
        end
    end
    % 
    POS_store_omc{item}.pos_accm=delta_pos_norm_accm;
    POS_store_omc{item}.pos_accuracy=POS_store_omc{item}.pos_err_norm./delta_pos_norm_accm(end);
    POS_store_omc{item}.pos_accuracyf=POS_store_omc{item}.pos_errf_norm./delta_pos_norm_accm(end);
    POS_store_omc{item}.t=0:0.01:(nlen-1)*0.01;
end

figure
hold on
plot3(POS_store_omc{2}.pos_errf(:,1),POS_store_omc{2}.pos_errf(:,2),POS_store_omc{2}.pos_errf(:,3),'-o','LineWidth',2,...
    'Color','red','MarkerIndices',1:10:length(pos_omc_nimu(:,1)),'LineWidth',2)
hold on
%plot3(POS_store_omc{1}.pos_err(:,1),POS_store_omc{1}.pos_err(:,2),POS_store_omc{1}.pos_err(:,3),'LineWidth',2,'color','g')
plot3(POS_store_omc{2}.pos_err(:,1),POS_store_omc{2}.pos_err(:,2),POS_store_omc{2}.pos_err(:,3),'LineWidth',2,'color','black','Marker','+','MarkerIndices',1:10:length(POS_store{2}.pos(:,1)))
%plot3(POS_store_omc{3}.pos_err(:,1),POS_store_omc{3}.pos_err(:,2),POS_store_omc{3}.pos_err(:,3),'LineWidth',2,'color','blue')
%plot3(POS_store_omc{4}.pos_err(:,1),POS_store_omc{4}.pos_err(:,2),POS_store_omc{4}.pos_err(:,3),'LineWidth',2,'color','m')
%plot3(POS_store_omc{5}.pos_err(:,1),POS_store_omc{5}.pos_err(:,2),POS_store_omc{5}.pos_err(:,3),'LineWidth',2,'color',[0.4940 0.1840 0.5560])
%plot3(POS_store_omc{6}.pos(:,1),POS_store_omc{6}.pos(:,2),POS_store_omc{6}.pos(:,3),'LineWidth',2,'color',[0.8500 0.3250 0.0980])
xlabel('x (m)','interpreter','latex')
ylabel('y (m)','interpreter','latex')
zlabel('z (m)','interpreter','latex')
view(-87,15)
legend('OMC','EKS','MKCERTS','ESKF','GD','DOE','interpreter','latex','Location','southwest')
set(gca,'fontsize',16)
set(gcf,'position',[100 100 750 600])

%% we calculate the error with repect to the whole trajectory length
item=2;
figure
box on
yyaxis left
hold on
plot(POS_store_omc{item}.t,POS_store_omc{item}.pos_err_norm,'LineWidth',2,'Color','red')
plot(POS_store_omc{item}.t,POS_store_omc{item}.pos_errf_norm,'LineWidth',2,'Color','blue')

%plot(POS_store_omc{item}.pos_accm,'LineWidth',2,'Color','black')
set(gca,'fontsize',16)
ylabel('m','interpreter','latex')
legend('$\|p_e\|$ (open)','$\|p_e\|$ (closed)','interpreter','latex')
ylim([0,max(POS_store_omc{item}.pos_errf_norm)])
yyaxis right
hold on
plot(POS_store_omc{item}.t,POS_store_omc{item}.pos_accuracy*100,'+','Markersize',8,'MarkerIndices',1:50:length(POS_store_omc{item}.t),'MarkerEdgeColor','black','LineWidth',1)
plot(POS_store_omc{item}.t,POS_store_omc{item}.pos_accuracyf*100,'+','Markersize',8,'MarkerIndices',1:50:length(POS_store_omc{item}.t),'MarkerEdgeColor','m','LineWidth',1)
set(gca,'fontsize',16)
xlabel('time (s)','interpreter','latex')
legend('error (closed)','error (open)','error rate (closed)','error rate (open)','interpreter','latex')
ylim([0,max(POS_store_omc{item}.pos_accuracyf)]*100)
ylabel('error rate ($\%$)','interpreter','latex')
set(gcf,'position',[100 100 750 600])
xlim([POS_store_omc{item}.t(1) POS_store_omc{item}.t(end)])


item=2;
poserrmax1=max(POS_store_omc{item}.pos_err_norm);
poserrmax2=max(POS_store_omc{item}.pos_errf_norm);

(poserrmax2-poserrmax1)/poserrmax2

poserr1=rms(POS_store_omc{item}.pos_err_norm);
poserr2=rms(POS_store_omc{item}.pos_errf_norm);

%% step by step trajectory comparison: align the sampling rate
Acc=Accelerometer(index_imu,:);
Acc=Acc(1:4:end,:);
Gyr=Gyroscope(index_imu,:);
Gyr=Gyr(1:4:end,:);
Mag=Magnetic(index_imu,:);
Mag=Mag(1:4:end,:);
freeAccInd=freeAcc_ceks(index_imu,:);
freeAccInd=freeAccInd(1:4:end,:);
index=1:length(Mag);
time=1:length(Mag);
time=time';
mydataInd=[time,Acc,Gyr,Mag];
PIndex=gyroscope_norm(mydataInd,freeAccInd,index,fs/4); % the moveing interval except the first row
%PIndex=gyroscope_norm_sampling100(mydataInd,freeAccInd); % the moveing interval except the first row
% we only care about the trajectory in the moving region
pos_omc_step=nan(size(pos_omc));
for item=1:6
POS_store_step{item}.pos=nan(size(POS_store_omc{item}.pos));
POS_store_step{item}.P=nan(size(POS_store_omc{item}.P)); % covariance
[seg,col]=size(PIndex);
for i=1:seg-1
    index=PIndex(i,2):PIndex(i+1,1);
    %% %% %% %%
    % pos forward
    offset=POS_store_omc{item}.pos(index(1),:)-pos_omc(index(1),:);
    POS_store_step{item}.pos(index,:)=POS_store_omc{item}.pos(index,:)-offset;
    pos_omc_step(index,:)=pos_omc(index,:);
    % covariance forward
    POS_store_step{item}.P(index,:)=POS_store_omc{item}.P(index,:)-POS_store_omc{item}.P(index(1),:);
    %% %% %% %% 
end
end 
% trajectory estimation with reset at each step
figure
hold on
plot3(pos_omc_step(:,1),pos_omc_step(:,2),pos_omc_step(:,3),'-o','LineWidth',2,...
    'Color','red','MarkerIndices',1:10:length(pos_omc_step(:,1)),'LineWidth',2)
hold on
plot3(POS_store_step{1}.pos(:,1),POS_store_step{1}.pos(:,2),POS_store_step{1}.pos(:,3),'LineWidth',2,'color','g')
plot3(POS_store_step{2}.pos(:,1),POS_store_step{2}.pos(:,2),POS_store_step{2}.pos(:,3),'LineWidth',2,'color','black','Marker','+','MarkerIndices',1:10:length(POS_store{2}.pos(:,1)))
plot3(POS_store_step{3}.pos(:,1),POS_store_step{3}.pos(:,2),POS_store_step{3}.pos(:,3),'LineWidth',2,'color','blue')
plot3(POS_store_step{4}.pos(:,1),POS_store_step{4}.pos(:,2),POS_store_step{4}.pos(:,3),'LineWidth',2,'color','m')
plot3(POS_store_step{5}.pos(:,1),POS_store_step{5}.pos(:,2),POS_store_step{5}.pos(:,3),'LineWidth',2,'color',[0.4940 0.1840 0.5560])
%plot3(POS_store{6}.pos(:,1),POS_store{6}.pos(:,2),POS_store{6}.pos(:,3),'LineWidth',2,'color',[0.8500 0.3250 0.0980])
xlabel('x (m)','interpreter','latex')
ylabel('y (m)','interpreter','latex')
zlabel('z (m)','interpreter','latex')
view(-113,55)
legend('OMC','EKS','MKCERTS','ESKF','GD','DOE','interpreter','latex','Location','southwest')
set(gca,'fontsize',16)
set(gcf,'position',[100 100 750 600])


%% error trajectory
for item=1:6
    POS_store_step{item}.err=POS_store_step{item}.pos-pos_omc_step; % error trajectory
    POS_store_step{item}.err_new=POS_store_step{item}.err; % new error trajectory
    indexnan=find(isnan(POS_store_step{item}.err(:,1))); % nan index
    POS_store_step{item}.err_new(indexnan,:)=[]; % set nan to []
    POS_store_step{item}.err_new_rms=rms(POS_store_step{item}.err_new); % rms
    POS_store_step{item}.err_new_max=max(abs(POS_store_step{item}.err_new)); % max
    POS_store_step{item}.Pnew=POS_store_step{item}.P; %
    POS_store_step{item}.Pnew(indexnan)=0; %
end


%% patch figure
sigma3_ceks=3*sqrt(abs(POS_store_step{2}.Pnew));
cur_omc=pos_omc;
tstep=0:0.01:(length(cur_omc)-1)*0.01;
cur=POS_store_step{2}.pos; % cekfs 
%
figure
hold on
x1=subplot(3,1,1);
hold on
plot(tstep,cur_omc(:,1),'Color','red','LineWidth',1,'LineStyle','-')
plot(tstep,cur(:,1),'Color','black','LineWidth',1.5,'LineStyle','-')
hold on
plot(tstep,cur(:,1)+sigma3_ceks,'Color','m','LineStyle','--','LineWidth',1.5)
plot(tstep,cur(:,1)-sigma3_ceks,'Color','g','LineStyle','--','LineWidth',1.5)
ylabel('x (m)','interpreter','latex')
legend('OMC','MKCERTS','MKCERTS UB','MKCERTS LB','orientation','horizontal','fontsize',12)
box on
xticks([])
set(gca,'fontsize',16)
x2=subplot(3,1,2);
hold on
plot(tstep,cur_omc(:,2),'Color','red','LineWidth',1,'LineStyle','-')
plot(tstep,cur(:,2),'Color','black','LineWidth',1.5,'LineStyle','-')
plot(tstep,cur(:,2)+sigma3_ceks,'Color','m','LineStyle','--','LineWidth',1.5)
plot(tstep,cur(:,2)-sigma3_ceks,'Color','g','LineStyle','--','LineWidth',1.5)
ylabel('y (m)','interpreter','latex')
xticks([])
set(gca,'fontsize',16)
box on
x3=subplot(3,1,3);
hold on
plot(tstep,cur_omc(:,3),'Color','red','LineWidth',1,'LineStyle','-')
plot(tstep,cur(:,3),'Color','black','LineWidth',1.5,'LineStyle','-')
plot(tstep,cur(:,3)+sigma3_ceks,'Color','m','LineStyle','--','LineWidth',1.5)
plot(tstep,cur(:,3)-sigma3_ceks,'Color','g','LineStyle','--','LineWidth',1.5)
xlabel('time (s)','interpreter','latex')
ylabel('z (m)','interpreter','latex')
set(gcf,'position',[100 100 750 600])
set(gca,'fontsize',16)
linkaxes([x1,x2,x3],'x')
xlim([tstep(1),tstep(end)]);
box on

%% 
fprintf('EKS& %.3f & %.3f &%.3f ',error.err_eks_rms(1),error.err_eks_rms(2),error.err_eks_rms(3));
fprintf('&%.3f & %.3f &%.3f',POS_store_step{1}.err_new_rms(1),POS_store_step{1}.err_new_rms(2),POS_store_step{1}.err_new_rms(3));
fprintf('&%.3f & %.3f &%.3f\r',POS_store_step{1}.err_new_max(1),POS_store_step{1}.err_new_max(2),POS_store_step{1}.err_new_max(3));

fprintf('MKCERTS& %.3f & %.3f &%.3f ',error.err_ceks_rms(1),error.err_ceks_rms(2),error.err_ceks_rms(3));
fprintf('&%.3f & %.3f &%.3f',POS_store_step{2}.err_new_rms(1),POS_store_step{2}.err_new_rms(2),POS_store_step{2}.err_new_rms(3));
fprintf('&%.3f & %.3f &%.3f\r',POS_store_step{2}.err_new_max(1),POS_store_step{2}.err_new_max(2),POS_store_step{2}.err_new_max(3));

fprintf('ESKF& %.3f & %.3f &%.3f ',error.err_eskf_rms(1),error.err_eskf_rms(2),error.err_eskf_rms(3));
fprintf('&%.3f & %.3f &%.3f',POS_store_step{3}.err_new_rms(1),POS_store_step{3}.err_new_rms(2),POS_store_step{3}.err_new_rms(3));
fprintf('&%.3f & %.3f &%.3f\r',POS_store_step{3}.err_new_max(1),POS_store_step{3}.err_new_max(2),POS_store_step{3}.err_new_max(3));

fprintf('GD& %.3f & %.3f &%.3f ',error.err_gd_rms(1),error.err_gd_rms(2),error.err_gd_rms(3));
fprintf('&%.3f & %.3f &%.3f',POS_store_step{4}.err_new_rms(1),POS_store_step{4}.err_new_rms(2),POS_store_step{4}.err_new_rms(3));
fprintf('&%.3f & %.3f &%.3f\r',POS_store_step{4}.err_new_max(1),POS_store_step{4}.err_new_max(2),POS_store_step{4}.err_new_max(3));

fprintf('DOE& %.3f & %.3f &%.3f ',error.err_doe_rms(1),error.err_doe_rms(2),error.err_doe_rms(3));
fprintf('&%.3f & %.3f &%.3f',POS_store_step{5}.err_new_rms(1),POS_store_step{5}.err_new_rms(2),POS_store_step{5}.err_new_rms(3));
fprintf('&%.3f & %.3f &%.3f\r',POS_store_step{5}.err_new_max(1),POS_store_step{5}.err_new_max(2),POS_store_step{5}.err_new_max(3));

% fprintf('MKCERTS: %.3f \r',POS_store_step{2}.err_new_rms);
% fprintf('ESKF: %.3f \r',POS_store_step{3}.err_new_rms);
% fprintf('GD: %.3f \r',POS_store_step{4}.err_new_rms);
% fprintf('DOE: %.3f \r',POS_store_step{5}.err_new_rms);
% fprintf('MKMC: %.3f \r',POS_store_step{6}.err_new_rms);

fprintf('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
fprintf('\r')

fprintf('EKS& %.3f & %.3f &%.3f ',error.err_eks_rms(1),error.err_eks_rms(2),error.err_eks_rms(3));
fprintf('&%.3f & %.3f &%.3f\r',POS_store_step{1}.err_new_rms(1),POS_store_step{1}.err_new_rms(2),POS_store_step{1}.err_new_rms(3));

fprintf('MKCERTS& %.3f & %.3f &%.3f ',error.err_ceks_rms(1),error.err_ceks_rms(2),error.err_ceks_rms(3));
fprintf('&%.3f & %.3f &%.3f\r',POS_store_step{2}.err_new_rms(1),POS_store_step{2}.err_new_rms(2),POS_store_step{2}.err_new_rms(3));

fprintf('ESKF& %.3f & %.3f &%.3f ',error.err_eskf_rms(1),error.err_eskf_rms(2),error.err_eskf_rms(3));
fprintf('&%.3f & %.3f &%.3f\r',POS_store_step{3}.err_new_rms(1),POS_store_step{3}.err_new_rms(2),POS_store_step{3}.err_new_rms(3));

fprintf('GD& %.3f & %.3f &%.3f ',error.err_gd_rms(1),error.err_gd_rms(2),error.err_gd_rms(3));
fprintf('&%.3f & %.3f &%.3f\r',POS_store_step{4}.err_new_rms(1),POS_store_step{4}.err_new_rms(2),POS_store_step{4}.err_new_rms(3));

fprintf('DOE& %.3f & %.3f &%.3f ',error.err_doe_rms(1),error.err_doe_rms(2),error.err_doe_rms(3));
fprintf('&%.3f & %.3f &%.3f\r',POS_store_step{5}.err_new_rms(1),POS_store_step{5}.err_new_rms(2),POS_store_step{5}.err_new_rms(3));

%% animation %%

Quat_ceks=compact(Quat_ceks);

Quat_ceks_algin=Quat_ceks(index_imu,:);
Quat_ceks_algin=Quat_ceks_algin(1:4:end,:);
posPlot=POS_store_omc{item}.pos;


quat2Matrix=quatern2rotMat(Quat_ceks_algin);
fs=100;
samplePeriod=1/fs;
% Create 6 DOF animation
SamplePlotFreq = 2;
Spin = 120;
% 'Trail', 'All'
SixDofAnimation(posPlot, quat2Matrix, ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
                'Position', [9 39 900 700], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
                'CreateAVI', false, 'AVIfileName','3','AVIfileNameEnum', true, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));

view(41.5,21)



end