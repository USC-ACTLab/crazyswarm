%B = 8000; % something weird with initial data
B = 12000;
E = length(t)-2000;
sel = @(x) x(:,B:E);
dt = (t(2) - t(1)) / 1000;
IMU_AHEAD = 10;

t = t(B:E);

acc_imu = 9.81 * [acc_x acc_y acc_z]';
gyro_imu = (pi / 180) * [gyro_x gyro_y gyro_z]';

gyro_error = mean(gyro_imu(:,1:1000), 2);
acc_error = mean(acc_imu(:,1:1000), 2) - [0 0 9.81]';

acc_imu = sel(acc_imu);
gyro_imu = sel(gyro_imu);
quat_vicon = sel([q0 q1 q2 q3]');
pos_vicon = sel([x y z]');

acc_imu = acc_imu(:,1:(end-IMU_AHEAD));
gyro_imu = gyro_imu(:,1:(end-IMU_AHEAD));
t = t(1:(end-IMU_AHEAD));
pos_vicon = pos_vicon(:,(1+IMU_AHEAD):end);
quat_vicon = quat_vicon(:,(1+IMU_AHEAD):end);

acc_unbias = bsxfun(@minus, acc_imu, acc_error);
gyro_unbias = bsxfun(@minus, gyro_imu, gyro_error);

[b,a] = butter(2, 0.01);
acc_filter = acc_unbias;
gyro_filter = gyro_unbias;
for i=1:3
    acc_filter(i,:) = filter(b,a,acc_unbias(i,:));
    gyro_filter(i,:) = filter(b,a,gyro_unbias(i,:));
end

whiten = @(x) bsxfun(@minus, x, mean(x, 2));
acc_white = whiten(acc_imu);
acc_white(3,:) = acc_white(3,:) + 9.81;
gyro_white = whiten(gyro_imu);

quat_eye = repmat([0; 0; 0; 1], 1, E);
acc_zero = 0 * acc_imu;
acc_zero(3,:) = 9.81;
gyro_zero = 0 * gyro_imu;
