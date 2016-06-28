fname = 'moving7.mat';
load(fname);
process;

NPTS = 1000;
NPTS = size(pos_vicon, 2);
[p_ekf, v_ekf, a_ekf, q_ekf, bw_ekf, ba_ekf] = ...
    ekf_full(acc_unbias, gyro_unbias, pos_vicon, quat_vicon, dt);
%if any(isnan(p_ekf(:))) || any(isnan(v_ekf(:))) || any(isnan(q_ekf(:)))
%    error('NaN output');
%end
[p_c, v_c, q_c, ~, ~, P_c] = ...
	ekfmex(t', acc_unbias, gyro_unbias, pos_vicon, quat_vicon, []);

p_test = p_ekf;
q_test = q_ekf;

% 3d plot
clf; hold on;
p_test = p_test(:,1:NPTS);
pos_vicon = pos_vicon(:,1:NPTS);
plot3n(p_test);
plot3n(pos_vicon);
legend('ekf', 'vicon');

% compute time point with most error and plot with markers
axis equal;
p_err = p_test - pos_vicon;
p_rmse = sqrt(sum(p_err .^ 2, 1));
[max_rmse, iworst] = max(p_rmse);
fprintf('\nmax rmse: %f\n', max_rmse);
if ~exist('best_rmse', 'var') || max_rmse < best_rmse
    best_rmse = max_rmse;
    fprintf('new high score!\n');
end
plot3(p_test(1,iworst), p_test(2,iworst), p_test(3,iworst), 'b*');
plot3(pos_vicon(1,iworst), pos_vicon(2,iworst), pos_vicon(3,iworst), 'r*');

%return;

% plot quaternion imaginary components
clf;
ind = 1:6000;
q_m = mahony(quat_vicon(:,1), gyro_unbias, acc_unbias, dt);
for i=1:1
    %subplot(3,1,i);
    hold on;
    plot(q_ekf(i,ind));
    plot(quat_vicon(i,ind));
    %plot(q_m(i,ind), 'g');
    legend('ekf', 'vicon', 'mahony');
end