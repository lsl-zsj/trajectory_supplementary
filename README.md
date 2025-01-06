This work proposes a multi-kernel correntropy Kalman smoother for orientation estimation and considers position constraints for trajectory estimation using foot-mounted IMUs. We prove that the maximum positioning error can be reduced to 25% of the original values by involving a single loop closure, and it can be reduced to a designated value by designing multiple position constraints properly.

Requirement: MATLAB 2023b
Example codes and descriptions: 
1: smoother_impulsive_measurement.m under folder "Simulation": Performance comparison of different smoothers
2: walking_square.m under folder "Square_walking_OPC": square walking compared with optical motion capture (OPC) data
3: stair_walking_pilot_s3_intersection_rfoot_lfoot.m under the folder "Stair_outdoor":  walking on a long stair.
4: stair_slope_walking_rfoot_lfoot.m under the folder "Stair_slope_walking":  square walking passing a stair and slope 
5: stair_walking_rfoot_lfoot.m under the folder "Stair_walking": square walking passing a stair. 
6: square_walking_rfoot_lfoot.m under folder "Square_walking": square walking.
