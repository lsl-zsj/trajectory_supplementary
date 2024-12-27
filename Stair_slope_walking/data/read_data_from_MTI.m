function read_data_from_MTI()

clear all
clc

%% stair slope walking 1
filename1='MT_0080001028_031_rf_stair_slope_1-000.txt';
filename2='MT_0080001039_004_lf_stair_slope_1-000.txt';
filename3='MT_0080001032_004_wt_stair_slope_1-000.txt';
savename='IMU_STAIR_SLOPE_1';

xsens_data_handle(filename1,filename2,filename3,savename);

%% stair slope walking 2
filename1='MT_0080001028_033_rf_stair_slope_2-000.txt';
filename2='MT_0080001039_005_lf_stair_slope_2-000.txt';
filename3='MT_0080001032_005_wt_stair_slope_2-000.txt';
savename='IMU_STAIR_SLOPE_2';

xsens_data_handle(filename1,filename2,filename3,savename);



end