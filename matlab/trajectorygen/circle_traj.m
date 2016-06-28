function [knot, soln, free] = circle_traj(helipad, radius)
	
	npts = 8;
	npieces = npts - 1;

    degree = 7;
    continuity = 4;
	dim = 4;

	% mission with only position constraints
	x = nan(3, npts, 4);

	% start on ground with 0 vel, 0 acc at theta=0
    x(:,1,:) = 0;         % initial zeros in everything up to acceleration
	x(1,1,:) = [1 0 0 0];

	x(:,2,:) = 0;         % take off, come to rest again
	x(1,2,:) = [1 0 1 0];

	x(:,3,:) = x(:,2,:);         % stabilization period. don't move

	for i=4:8
		theta = (pi/2) * (i-3);
		x(1,i,:) = [cos(theta) sin(theta) 1 0];  % position
	end

	t_takeoff = 2;
	t_settle = 2;
	t_circle = 1;
	intervals = [t_takeoff t_settle t_circle * ones(1,5)];
	knot = [0 cumsum(intervals)];

	[soln, free] = pp_waypoints(knot, degree, continuity, x, 'monomial');
	fprintf('%d free dimensions\n', size(free, 2));

	pp = polyvec2pp(knot, dim, soln);
end
