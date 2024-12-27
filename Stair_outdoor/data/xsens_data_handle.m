function xsens_data_handle(filename1,filename2,savename)

%IMU.Coordinate='ENU';
close all
clear all
filename1='MT_0080001028_029_right_3-000.txt';
imu1 = readtable(filename1);

filename2='MT_0080001039_002_left_3-000.txt';
imu2 = readtable(filename2);

savename='IMU_STAIR_3';
% imu1 data
IMU1.UTC=[imu1.UTC_Year,imu1.UTC_Month,imu1.UTC_Day,imu1.UTC_Hour,imu1.UTC_Minute,imu1.UTC_Second,imu1.UTC_Nano/10^(9)];
IMU1.UTC_secnod=imu1.UTC_Minute*60+imu1.UTC_Second+imu1.UTC_Nano/10^(9);
IMU1.count=imu1.PacketCounter;
IMU1.fs=400;
IMU1.Acceleration=[imu1.Acc_X,imu1.Acc_Y,imu1.Acc_Z];
IMU1.Gyroscope=[imu1.Gyr_X,imu1.Gyr_Y,imu1.Gyr_Z];
IMU1.Magnetic=[imu1.Mag_X,imu1.Mag_Y,imu1.Mag_Z];
len=length(IMU1.Acceleration);
%IMU.Coordinate='ENU';
IMU1.FreeAcc=[imu1.FreeAcc_E,imu1.FreeAcc_N,imu1.FreeAcc_U];
IMU1.quat=[imu1.Quat_q0,imu1.Quat_q1,imu1.Quat_q2,imu1.Quat_q3];
IMU1.Euler=[imu1.Roll,imu1.Pitch,imu1.Yaw];
Quat=quaternion(IMU1.quat);
EulerNew=eulerd(Quat,'ZXY','frame');
IMU1.EulerNew=EulerNew;
% delete first nan element and first magnetic data 
index_nan=0;
index_nan_end=0;
for i=1:len
    if(~isnan(IMU1.Magnetic(i,1)))
        index_nan=i;
        break;
    end
end
for i=len:-1:1
    if(~isnan(IMU1.Magnetic(i,1)))
        index_nan_end=i;
        break;
    end
end
index=[1:index_nan,index_nan_end+1:len];
IMU1.UTC(index,:)=[];
IMU1.UTC_secnod(index)=[];
IMU1.count(index)=[];
IMU1.Acceleration(index,:)=[];
IMU1.Gyroscope(index,:)=[];
IMU1.Magnetic(index,:)=[];
IMU1.FreeAcc(index,:)=[];
IMU1.quat(index,:)=[];
IMU1.Euler(index,:)=[];
IMU1.EulerNew(index,:)=[];
% aligne the sample rate of accelerometer and magnetometer
lenn=length(IMU1.count);
for i=1:lenn/4
    for j=1:3
    IMU1.Magnetic((i-1)*4+j,:) = IMU1.Magnetic((i)*4,:);
    end
end

%% imu2 data
IMU2.UTC=[imu2.UTC_Year,imu2.UTC_Month,imu2.UTC_Day,imu2.UTC_Hour,imu2.UTC_Minute,imu2.UTC_Second,imu2.UTC_Nano/10^(9)];
IMU2.UTC_secnod=imu2.UTC_Minute*60+imu2.UTC_Second+imu2.UTC_Nano/10^(9);
IMU2.count=imu2.PacketCounter;
IMU2.fs=400;
IMU2.Acceleration=[imu2.Acc_X,imu2.Acc_Y,imu2.Acc_Z];
IMU2.Gyroscope=[imu2.Gyr_X,imu2.Gyr_Y,imu2.Gyr_Z];
IMU2.Magnetic=[imu2.Mag_X,imu2.Mag_Y,imu2.Mag_Z];
len=length(IMU2.Acceleration);
%IMU.Coordinate='ENU';
IMU2.FreeAcc=[imu2.FreeAcc_E,imu2.FreeAcc_N,imu2.FreeAcc_U];
IMU2.quat=[imu2.Quat_q0,imu2.Quat_q1,imu2.Quat_q2,imu2.Quat_q3];
IMU2.Euler=[imu2.Roll,imu2.Pitch,imu2.Yaw];
Quat=quaternion(IMU2.quat);
EulerNew=eulerd(Quat,'ZXY','frame');
IMU2.EulerNew=EulerNew;
% delete first nan element and first magnetic data 
index_nan=0;
index_nan_end=0;
for i=1:len
    if(~isnan(IMU2.Magnetic(i,1)))
        index_nan=i;
        break;
    end
end
for i=len:-1:1
    if(~isnan(IMU2.Magnetic(i,1)))
        index_nan_end=i;
        break;
    end
end
index=[1:index_nan,index_nan_end+1:len];
IMU2.UTC(index,:)=[];
IMU2.UTC_secnod(index)=[];
IMU2.count(index)=[];
IMU2.Acceleration(index,:)=[];
IMU2.Gyroscope(index,:)=[];
IMU2.Magnetic(index,:)=[];
IMU2.FreeAcc(index,:)=[];
IMU2.quat(index,:)=[];
IMU2.Euler(index,:)=[];
IMU2.EulerNew(index,:)=[];
% aligne the sample rate of accelerometer and magnetometer
lenn=length(IMU2.count);
for i=1:lenn/4
    for j=1:3
    IMU2.Magnetic((i-1)*4+j,:) = IMU2.Magnetic((i)*4,:);
    end
end
%


%% sync the imu1 and imu2 based on UTC time
% 
imu1_big=zeros(2,1); % assume that utcinit1>utcinit2
initsecond1=IMU1.UTC_secnod(1);
initsecond2=IMU2.UTC_secnod(1);

endsecond1=IMU1.UTC_secnod(end);
endsecond2=IMU2.UTC_secnod(end);

if(initsecond1>initsecond2)
    imu1_big(1)=1;
else
    imu1_big(1)=0;
end
if(endsecond1<endsecond2)
    imu1_big(2)=1;
else
    imu1_big(2)=0;
end

% imu1_big=[1 0] % init time> imu1   final time < imu2
% init time
if(imu1_big(1)==1)
    ts=ceil(initsecond1);
else
    ts=ceil(initsecond2);
end
% final time
if(imu1_big(2)==1)
    te=floor(endsecond1);
else
    te=floor(endsecond2);
end

index1=find(IMU1.UTC_secnod>=ts&IMU1.UTC_secnod<=te);
index2=find(IMU2.UTC_secnod>=ts&IMU2.UTC_secnod<=te);


IMU1.index1=index1;
IMU2.index2=index2;

% raw data plot
figure
hold on
plot(IMU1.Acceleration(index1,1),'linewidth',2,'color','red','LineStyle','-')
plot(IMU2.Acceleration(index2,1),'linewidth',2,'color','red','LineStyle','--')

plot(IMU1.Acceleration(index1,2),'linewidth',2,'color','black','LineStyle','-')
plot(IMU2.Acceleration(index2,2),'linewidth',2,'color','black','LineStyle','--')

plot(IMU1.Acceleration(index1,3),'linewidth',2,'color','blue','LineStyle','-')
plot(IMU2.Acceleration(index2,3),'linewidth',2,'color','blue','LineStyle','--')

ylabel('Acc ($m /s^2 $)','interpreter','latex')


% 
% figure
% plot(IMU1.EulerNew,'DisplayName','imuMC.Euler')
% 

IMU.right=IMU1;
IMU.left=IMU2;

save(savename, 'IMU')



end