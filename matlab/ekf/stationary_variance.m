load 'stationary.mat';
process;

cov_acc = cov(acc_imu')
cov_gyro = cov(gyro_imu')
cov_pos_vicon = cov(pos_vicon')
cov_quat_vicon = cov(quat_vicon')