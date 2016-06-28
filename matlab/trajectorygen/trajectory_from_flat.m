function [rotmtx_imu2world, quat_imu2world, rpy_imu2world, omega] = trajectory_from_flat(xyz, vel, acc, jerk, yaw, d_yaw, mass)
	FEASIBLE_THRUST_TO_WEIGHT = 4.0; % generous, realistic is more like 1-2

	npts = size(xyz,2);

	% thrust vector with magnitude
	thrust = acc;
	g = 9.81;
	thrust(3,:) = thrust(3,:) + g;
	thrust_mag = mass * sqrt(sum(thrust.^2, 1)); % newtons
	peak_thrust_to_weight = max(thrust_mag) / (mass * g);
	if peak_thrust_to_weight > FEASIBLE_THRUST_TO_WEIGHT
		warning(['trajectory requres peak thrust-to-weight ratio of %f, '...
		         'which exceeds feasible limit of %f'],...
                peak_thrust_to_weight, FEASIBLE_THRUST_TO_WEIGHT);
	end

	% coordinate axes of quadcopter in world coords
	z_body = normc(thrust);
	x_world = [cos(yaw); sin(yaw); zeros(1,npts)];
	y_body = normc(cross(z_body, x_world));
	x_body = cross(y_body, z_body);
	rotmtx_imu2world = reshape([x_body; y_body; z_body], [3 3 npts]);
    %rotmtx = permute(rotmtx, [2 1 3]);
    
    % TODO convert to other rotation formats - or not??
    quat_imu2world = rot2quat(rotmtx_imu2world);    
    rpy_imu2world = quat2rpy(quat_imu2world);

	% angular velocities
	jerk_orth_zbody = jerk - bsxfun(@times, z_body, dot(z_body, jerk));
	h_w = bsxfun(@times, (mass ./ thrust_mag), jerk_orth_zbody);
    p = -dot(h_w, y_body);
    q = dot(h_w, x_body);
    r = d_yaw .* z_body(3,:);
	omega = [p; q; r];
end