function read_data_from_MTI()

clear all
clc

%% square walking 1
filename1='MT_0080001028_024_rf_square_1-000.txt';
filename2='MT_0080001039_000_lf_square_1-000.txt';
filename3='MT_0080001032_000_wt_square_1-000.txt';
savename='IMU_SQUARE_1';

xsens_data_handle(filename1,filename2,filename3,savename);

%% square walking 2

filename1='MT_0080001028_028_rf_square_2-000.txt';
filename2='MT_0080001039_001_lf_square_2-000.txt';
filename3='MT_0080001032_001_wt_square_2-000.txt';
savename='IMU_SQUARE_2';

xsens_data_handle(filename1,filename2,filename3,savename);



end