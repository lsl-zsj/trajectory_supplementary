function xsens_data_handle(filename1,filename2,filename3,savename)

%IMU.Coordinate='ENU';
% close all
% clear all

% filename1='MT_0080001028_024_rf_square_1-000.txt';
% filename2='MT_0080001039_000_lf_square_1-000.txt';
% filename2='MT_0080001032_000_wt_square_1-000.txt';
% savename='IMU_SQUARE_1';

imu1 = readtable(filename1);
imu2 = readtable(filename2);
imu3 = readtable(filename3);

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


%% imu3 data
IMU3.UTC=[imu3.UTC_Year,imu3.UTC_Month,imu3.UTC_Day,imu3.UTC_Hour,imu3.UTC_Minute,imu3.UTC_Second,imu3.UTC_Nano/10^(9)];
IMU3.UTC_secnod=imu3.UTC_Minute*60+imu3.UTC_Second+imu3.UTC_Nano/10^(9);
IMU3.count=imu3.PacketCounter;
IMU3.fs=400;
IMU3.Acceleration=[imu3.Acc_X,imu3.Acc_Y,imu3.Acc_Z];
IMU3.Gyroscope=[imu3.Gyr_X,imu3.Gyr_Y,imu3.Gyr_Z];
IMU3.Magnetic=[imu3.Mag_X,imu3.Mag_Y,imu3.Mag_Z];
len=length(IMU3.Acceleration);
%IMU.Coordinate='ENU';
IMU3.FreeAcc=[imu3.FreeAcc_E,imu3.FreeAcc_N,imu3.FreeAcc_U];
IMU3.quat=[imu3.Quat_q0,imu3.Quat_q1,imu3.Quat_q2,imu3.Quat_q3];
IMU3.Euler=[imu3.Roll,imu3.Pitch,imu3.Yaw];
Quat=quaternion(IMU3.quat);
EulerNew=eulerd(Quat,'ZXY','frame');
IMU3.EulerNew=EulerNew;
% delete first nan element and first magnetic data 
index_nan=0;
index_nan_end=0;
for i=1:len
    if(~isnan(IMU3.Magnetic(i,1)))
        index_nan=i;
        break;
    end
end
for i=len:-1:1
    if(~isnan(IMU3.Magnetic(i,1)))
        index_nan_end=i;
        break;
    end
end
index=[1:index_nan,index_nan_end+1:len];
IMU3.UTC(index,:)=[];
IMU3.UTC_secnod(index)=[];
IMU3.count(index)=[];
IMU3.Acceleration(index,:)=[];
IMU3.Gyroscope(index,:)=[];
IMU3.Magnetic(index,:)=[];
IMU3.FreeAcc(index,:)=[];
IMU3.quat(index,:)=[];
IMU3.Euler(index,:)=[];
IMU3.EulerNew(index,:)=[];
% aligne the sample rate of accelerometer and magnetometer
lenn=length(IMU3.count);
for i=1:lenn/4
    for j=1:3
    IMU3.Magnetic((i-1)*4+j,:) = IMU3.Magnetic((i)*4,:);
    end
end



%% sync the imu1 and imu2 based on UTC time
% 
initsecond1=IMU1.UTC_secnod(1);
initsecond2=IMU2.UTC_secnod(1);
initsecond3=IMU3.UTC_secnod(1);

endsecond1=IMU1.UTC_secnod(end);
endsecond2=IMU2.UTC_secnod(end);
endsecond3=IMU3.UTC_secnod(end);

ts=ceil(max([initsecond1,initsecond2,initsecond3]));
te=floor(min([endsecond1,endsecond2,endsecond3]));


index1=find(IMU1.UTC_secnod>=ts&IMU1.UTC_secnod<=te);
index2=find(IMU2.UTC_secnod>=ts&IMU2.UTC_secnod<=te);
index3=find(IMU3.UTC_secnod>=ts&IMU3.UTC_secnod<=te);


IMU1.index1=index1;
IMU2.index2=index2;
IMU3.index3=index3;

% raw data plot
figure
hold on
plot(IMU1.Acceleration(index1,1),'linewidth',2,'color','red','LineStyle','-')
plot(IMU2.Acceleration(index2,1),'linewidth',2,'color','red','LineStyle','--')
plot(IMU3.Acceleration(index3,1),'linewidth',2,'color','red','LineStyle',':')

plot(IMU1.Acceleration(index1,2),'linewidth',2,'color','black','LineStyle','-')
plot(IMU2.Acceleration(index2,2),'linewidth',2,'color','black','LineStyle','--')
plot(IMU3.Acceleration(index3,2),'linewidth',2,'color','black','LineStyle',':')

plot(IMU1.Acceleration(index1,3),'linewidth',2,'color','blue','LineStyle','-')
plot(IMU2.Acceleration(index2,3),'linewidth',2,'color','blue','LineStyle','--')
plot(IMU3.Acceleration(index3,3),'linewidth',2,'color','blue','LineStyle',':')

ylabel('Acc ($m /s^2 $)','interpreter','latex')


% 
% figure
% plot(IMU1.EulerNew,'DisplayName','imuMC.Euler')
% 

IMU.right=IMU1;
IMU.left=IMU2;
IMU.waist=IMU3;
save(savename, 'IMU')



end