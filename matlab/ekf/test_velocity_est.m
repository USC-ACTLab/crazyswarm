fname = 'moving7.mat';
load(fname);
process;
pos_vicon = fix_vicon_stairstep(pos_vicon);
ind = 1:1000;
pos_vicon = pos_vicon(:,ind);



N = size(pos_vicon, 2);
n_back = 5;
v_naive = diff(pos_vicon, 1, 2)/dt;
v_naive = v_naive(:,n_back:end);

v_good = NaN + v_naive;
for i=(n_back+1):N
	st = i - n_back;
	v_good(:,st) = estimate_velocity(pos_vicon(:,st:(i-1)), dt);
end

% plot quaternion imaginary components
clf;

for i=1:3
    subplot(3,1,i);
    hold on;
    plot(v_naive(i,:));
    plot(v_good(i,:));
    legend('naive', 'smooth');
end
