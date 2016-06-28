function [p_out, v_out, a_out, q_out, bw_out, ba_out] = ekf_full(acc, gyro, pos, quat, dt)


VICON_VAR_XY = 1.5e-9;
VICON_VAR_Z  = 1.0e-8;
VICON_VAR_Q  = 4.5e-6;
GYRO_VAR_XYZ = 0.2e-5;
ACC_VAR_XY   = 1.5e-5;
ACC_VAR_Z    = 3.9e-5;
ACC_VAR_XYZ  = 2.4e-5;
GRAV         = 9.81;

N = 15;
M = 6;

rep3 = @(x) [x x x]';
na = rep3(ACC_VAR_XY);
nba = 6e-4; % S.Weiss
nw = rep3(GYRO_VAR_XYZ);
nbw = nba / 10;


P = blkdiag(eye(9));
%P = load('Pinit_imuvis.mat');
%P = P.Pinit(1:9, 1:9);
% TEMP not using different var Z for synth. data
R = diag([VICON_VAR_XY VICON_VAR_XY VICON_VAR_XY VICON_VAR_Q VICON_VAR_Q VICON_VAR_Q]);

p_out = NaN + pos;
q_out = NaN + quat;
v_out = NaN + pos;
a_out = NaN + pos;
bw_out = a_out;
ba_out = a_out;

p = pos(:,1);
q = quat(:,1);
v = (pos(:,11) - pos(:,1)) / (10 * dt);
ba = zeros(3,1);
bw = ba;


IMU_HZ = 500;
VICON_HZ = 10;
VICON_SKIP = floor(IMU_HZ / VICON_HZ);
vicon_tick = 0;

npts = size(acc, 2);
n_vicons = 0;
for i=1:npts    
    omega = gyro(:,i);% - bw;
    acc_imu = acc(:,i);% - ba;

    %[p, v, q, P] = ekf_imu_mex(p, v, q, P, omega, acc_imu, dt);
	F = dynamic_matrix(q, omega, acc_imu, dt);
    [p, v, q] = propagate(p, v, q, omega, acc_imu, dt);
    Q = calcQ_mex(dt, q, omega, acc_imu, sqrt(na), sqrt(nba), sqrt(nw), sqrt(nbw));
    P = F * P * F' + Q;
    %P = (P + P') / 2;
    %assert_close(p, p2);
	%assert_close(v, v2);
	%assert_close(q, q2);
    
    %if ~issymmetric(P)
    %    assert(issymmetric(P));
    %end
    
    p_out(:,i) = p;
    q_out(:,i) = q;
    v_out(:,i) = v;
    %a_out(:,i) = acc_world;
    bw_out(:,i) = bw;
    ba_out(:,i) = ba;


    % vicon update - rate limited
    vicon_tick = vicon_tick + 1;
    if mod(vicon_tick, VICON_SKIP) == 0
        if i > 3500
            sfad = 0;
        end
    
		%[p, v, q, P, dbg2] = vicon_update_mex(p, v, q, P, R, pos(:,i), quat(:,i));
		[p, v, q, P, dbg] = vicon_update(p, v, q, P, R, pos(:,i), quat(:,i));
		n_vicons = n_vicons + 1;
        %assert_close(dbg, dbg2');
		%assert_close(p, p2);
		%assert_close(v, v2);
		%assert_close(q, q2);
		%assert_close(P, P2);
	end
end
n_vicons_decomposed = n_vicons
end

function assert_close(a, b)
    err = abs(a - b);
    assert(~any(err(:) > 0.00001));
end

function [p, v, q] = propagate(p, v, q, omega, acc, dt)
    q = qnorm(quat_gyro_update(q, omega, dt));
    
    acc_world = qvrot(q, acc);
    acc_world(3) = acc_world(3) - 9.81;
        
    v = v + dt * acc_world;
    p = p + dt * v;
end

function [p, v, q, P, dbg] = vicon_update(p, v, q, P, R, p_vicon, q_vicon)
    q_residual = qqmul(qinv(q), q_vicon);
    err_quat = 2 * q_residual(1:3) / (q_residual(4));
    err_pos = p_vicon - p;
    residual = [err_pos; err_quat];

    H = zeros(6, 9);
    H(1:3,1:3) = eye(3);
    H(4:6,7:9) = eye(3);
    
    S = H * P * H' + R;
    %K = P * H' / S;
    K = P * H' * cholinv_mex(S);
    
    dbg = H;

    correction = K * residual;
    p = p + correction(1:3);
    v = v + correction(4:6);
    error_quat = qrpy_small(correction(7:9));
    q = qnorm(qqmul(q, error_quat));
    %bw = bw + correction(10:12);
    %ba = ba + correction(13:15);

    IMKH = eye(9) - K * H;
    P = IMKH * P * IMKH' + K * R * K';
    %P = 0.5 * (P + P'); % TODO: need this?
end


function qq = qinv(q)
    qq = [-q(1) -q(2) -q(3) q(4)]';
end

function qq = quat_gyro_update(q, omega, dt)
    % from Trawny 2005 MARRS
	x = (dt / 2) * omega(1);
	y = (dt / 2) * omega(2);
	z = (dt / 2) * omega(3);

    OMEGA = [0 z -y x; -z 0 x y; y -x 0 z; -x -y -z 0];
    qq = (eye(4) + OMEGA) * q;
end

function qq = qnorm(q)
    qq = q ./ norm(q);
end

function q = qrpy_small(rpy)
    % from S. Weiss ethzasl_sensor_fusion eigen_utils.h
	q2 = sum(rpy.^2) / 4;
	if (q2 < 1) 
		q = [rpy/2; sqrt(1 - q2)];
	else
		w = 1 / sqrt(1 + q2);
		q = [rpy * (w/2); w];
	end
end

function qp = qqmul(q, p)
    % from Trawny 2005
	x =  q(4)*p(1) + q(3)*p(2) - q(2)*p(3) + q(1)*p(4);
	y = -q(3)*p(1) + q(4)*p(2) + q(1)*p(3) + q(2)*p(4);
	z =  q(2)*p(1) - q(1)*p(2) + q(4)*p(3) + q(3)*p(4);
    w = -q(1)*p(1) - q(2)*p(2) - q(3)*p(3) + q(4)*p(4);
	qp = [x y z w]';
end

function F = dynamic_matrix(q, omega, acc_imu, dt)
	dt_p2_2 = dt * dt * 0.5;
	dt_p3_6 = dt_p2_2 * dt / 3.0;
	dt_p4_24 = dt_p3_6 * dt * 0.25;
	dt_p5_120 = dt_p4_24 * dt * 0.2;

	C_eq = quat2rot(q);
	w_sk = skew(omega);
	a_sk = skew(acc_imu);

	Ca3 = C_eq * a_sk;
	eye3 = eye(3);
	A = Ca3 * (-dt_p2_2 * eye3 + dt_p3_6 * w_sk);% - dt_p4_24 * w_sk * w_sk);
	B = Ca3 * (dt_p3_6 * eye3 - dt_p4_24 * w_sk);% + dt_p5_120 * w_sk * w_sk);
	D = -A;
	E = eye3 - dt * w_sk;% + dt_p2_2 * w_sk * w_sk;
	FF = -dt * eye3 + dt_p2_2 * w_sk;% - dt_p3_6 * (w_sk * w_sk);
	C = Ca3 * FF;

	F = eye(9);
	F(1:3,4:6)   = dt * eye(3);
	F(1:3,7:9)   = A;
	%F(1:3,10:12) = B;
	%F(1:3,13:15) = -dt_p2_2 * C_eq;

	F(4:6,7:9)   = C;
	%F(4:6,10:12) = D;
	%F(4:6,13:15) = -dt * C_eq;

	F(7:9,7:9)   = E;
	%F(7:9,10:12) = FF;
end
