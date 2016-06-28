function find_delay
    fname = 'moving.mat';
    load(fname)
    process;

    pos_vicon = fix_vicon_stairstep(pos_vicon);
    acc_imu_simulated = accelerometer(pos_vicon, quat_vicon);
    
    MAX_SHIFT = 1000;
    mserrs = zeros(MAX_SHIFT + 1, 1);
    for shift=0:MAX_SHIFT
        acc_sim_shifted = acc_imu_simulated(:,(shift+1):end);
        acc_real_clipped = acc_imu(:,1:(end-shift));
        err = acc_sim_shifted - acc_real_clipped;
        mserr = mean(sqrt(sum(err.^2, 1)));
        mserrs(shift+1) = mserr;
    end

    plot(2 * (0:MAX_SHIFT), mserrs);
    xlabel('vicon delay hypothesis: milliseconds');
    ylabel('mean squared error');
end

function pp = fix_vicon_stairstep(pos)
	dp = diff(pos, 1, 2);
	flat = all(dp == 0, 1);
	changes = 1 + find(~flat);
	if changes(1) ~= 1
		changes = [1 changes];
    end
    if changes(end) ~= size(pos, 2)
        changes = [changes size(pos, 2)];
    end
	all_t = 1:size(pos,2);
	x = interp1(changes, pos(1,changes), all_t);
	y = interp1(changes, pos(2,changes), all_t);
	z = interp1(changes, pos(3,changes), all_t);
	pp = [x; y; z];
	assert(all(size(pp) == size(pos)));
    assert(~any(isnan(pp(:))));
end

function acc_imu = accelerometer(pos, quat)
	vel = diff(pos, 1, 2);
	vel = [vel(:,1) vel];
	acc_global = diff(vel, 1, 2);
	acc_global = [acc_global(:,1) acc_global];
	N = size(quat, 2);
	acc_imu = 0 * acc_global;
	for i=1:N
		a = acc_global(:,i);
		a(3) = a(3) + 9.81;
		acc_imu(:,i) = qvrot(quat(:,i), a);
	end
end

function qq = qinv(q)
    qq = [-q(1) -q(2) -q(3) q(4)]';
end

function qq = qvrot(q, v)
    qq = 2 * dot(q(1:3), v) * q(1:3) + ...
         (q(4)^2 - dot(q(1:3), q(1:3))) * v + ...
         2 * q(4) * cross(q(1:3), v);
end

function qq = quat_gyro_update(q, omega, dt)
	r = (dt / 2) * omega(1);
	p = (dt / 2) * omega(2);
	y = (dt / 2) * omega(3);

    qq = zeros(4,1);
	qq(1) =    q(1) + y*q(2) - p*q(3) + r*q(4);
	qq(2) = -y*q(1) +   q(2) + r*q(3) + p*q(4);
	qq(3) =  p*q(1) - r*q(2) +   q(3) + y*q(4);
	qq(4) = -r*q(1) - p*q(2) - y*q(3) +   q(4);
end

function qq = qnorm(q)
    qq = q ./ norm(q);
end

function q = qrpy_small(rpy)
	q2 = sum(rpy.^2) / 4;
	if (q2 < 1) 
		q = [rpy/2; sqrt(1 - q2)];
	else
		w = 1 / sqrt(1 + q2);
		q = [rpy * w/2; w];
	end
end

function qp = qqmul(q, p)
	x =    q(4)*p(1) + q(3)*p(2) - q(2)*p(3) + q(1)*p(4);
	y = -q(3)*p(1) +   q(4)*p(2) + q(1)*p(3) + q(2)*p(4);
	z =  q(2)*p(1) - q(1)*p(2) +   q(4)*p(3) + q(3)*p(4);
    w = -q(1)*p(1) - q(2)*p(2) - q(3)*p(3) +   q(4)*p(4);
	qp = [x y z w]';
end
