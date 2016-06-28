% make pp from artisanal hand-selected data points
load 'trim1.mat'
dt = 2.2 / 1000; % it was really 2.0, but i was moving the quadcopter by hand and did some nearly-infeasible accelerations
yaw_raw = quat2yaw(q);
xclick = xclick(1:(end-2)); % it was ending on high velocity
samples = [x(xclick) y(xclick) z(xclick) yaw_raw(xclick)]';
tclick = dt * xclick;
bspline = spapi(6, tclick, samples);
pp = fn2fm(bspline, 'pp');

% compute full trajectory from flat variables
npts = xclick(end) - xclick(1) + 1;
[xyz, vel, acc, rot_imu2w, q_imu2w, ~, omega, t] = trajectory_eval_piecewise(pp, 0.27, 9681);
assert(abs(t(2) - t(1) - dt) < 1e-15);
yaw = ppval(t, pp);
yaw = yaw(4,:);
N = size(yaw,2);

% sensor noise constants, measured
VICON_VAR_XY = 1.5e-9;
VICON_VAR_Z  = 1.0e-8;
VICON_VAR_Q  = 4.5e-6;%4.5e-7;
VICON_VAR_YAW = 2.14e-6;% stationary, probably much higher when moving
GYRO_VAR_XYZ = 0.2e-5;
ACC_VAR_XY   = 1.5e-5;
ACC_VAR_Z    = 3.9e-5;
ACC_VAR_XYZ  = 2.4e-5;
GRAV         = 9.81;
% made up
VAR_BIAS_ACC = 1e-12;
VAR_BIAS_GYR = 1e-15;

noise3 = @(variance) sqrt(variance) * randn(3,N);

% IMU biases as random walks
%bias_acc = cumsum(noise3(VAR_BIAS_ACC), 2);
%bias_gyr = cumsum(noise3(VAR_BIAS_GYR), 2);

% OR - IMU biases as constants
bias_acc = 1*repmat([0.1 -0.05 0.08]', 1, N);
bias_gyr = 1*repmat([0.04 0 -0.09]', 1, N);

% add noise and bias
pos_vicon = xyz + noise3(VICON_VAR_XY);
%yaw_vicon = yaw + sqrt(VICON_VAR_YAW) * randn(1,N);
% TODO figure out how to add noise to quaternion
quat_vicon = squeeze(q_imu2w);
quat_vicon(1:3,:) = -quat_vicon(1:3,:); % invert
quat_vicon = normc(quat_vicon + sqrt(VICON_VAR_Q) * randn(4,N)); % add noise
acc_imu = addgravityandrotateacc(acc, rot_imu2w) + bias_acc + noise3(ACC_VAR_XY);
gyr_imu = omega + bias_gyr + noise3(GYRO_VAR_XYZ);

% run ekf
% TODO make ekf that takes yaw
[p_ekf, v_ekf, a_ekf, q_ekf, bw_ekf, ba_ekf] = ...
	ekf_full(acc_imu, gyr_imu, pos_vicon, quat_vicon, dt);

% display results
hold off;
plot3n(p_ekf);
hold on;
plot3n(pos_vicon);
axis equal;
p_err = p_ekf - pos_vicon;
p_rmse = sqrt(sum(p_err .^ 2, 1));
[max_rmse, iworst] = max(p_rmse);
fprintf('\nmax rmse: %f\n', max_rmse);
fprintf('total rmse: %f\n', sum(p_rmse));
if ~exist('best_rmse', 'var') || max_rmse < best_rmse
    best_rmse = max_rmse;
    fprintf('new high score!\n');
end
plot3(p_ekf(1,iworst), p_ekf(2,iworst), p_ekf(3,iworst), 'b*');
plot3(pos_vicon(1,iworst), pos_vicon(2,iworst), pos_vicon(3,iworst), 'r*');
ba_final = ba_ekf(:,end)
bw_final = bw_ekf(:,end)
