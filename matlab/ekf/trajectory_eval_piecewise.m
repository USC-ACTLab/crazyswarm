function [xyz, vel, acc, rotmtx_imu2world, quat_imu2world, rpy_imu2world, omega, t] = trajectory_eval_piecewise(pp, mass, npts)
	assert(strcmp(pp.form, 'pp'));
	assert(pp.dim == 4);

	t0 = pp.breaks(1);
	t1 = pp.breaks(end);
	duration = t1 - t0;
	step = duration / (npts - 1);
	t = t0:step:t1;

	dpp = fnder(pp);
	d2pp = fnder(dpp);
	d3pp = fnder(d2pp);

	f = ppval(pp, t);
	df = ppval(dpp, t);
	d2f = ppval(d2pp, t);
	d3f = ppval(d3pp, t);

	xyz = f(1:3,:);
	vel = df(1:3,:);
	acc = d2f(1:3,:);
	jerk = d3f(1:3,:);

	yaw = f(4,:);
	d_yaw = df(4,:);

	[rotmtx_imu2world, quat_imu2world, rpy_imu2world, omega] = trajectory_from_flat(xyz, vel, acc, jerk, yaw, d_yaw, mass);
end
