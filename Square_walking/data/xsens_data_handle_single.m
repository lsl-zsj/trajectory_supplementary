function xsens_data_handle_single(filename1,savename)

%IMU.Coordinate='ENU';
close all
clear all
filename1='MT_0080001028_030_right_shift-000.txt';
imu1 = readtable(filename1);

savename='IMU_Shift';
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

IMU=IMU1;
save(savename, 'IMU')



end