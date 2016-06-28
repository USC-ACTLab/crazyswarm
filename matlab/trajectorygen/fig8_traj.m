function [pp, knot, soln, free] = fig8_traj(helipad, radius)
	
	npieces = 10;
	npts = npieces + 1;

    degree = 7;
    continuity = 4;
	dim = 4;

	syms tt;
	dt = 0.01;
	xvel = (-3*cos(tt-pi/2).^2*sin(tt-pi/2) -2*sin(tt-pi/2))/3;
	yvel = cos(2*(tt-pi/2));
	speed = sqrt(xvel.^2 + yvel.^2);
	speed_eval = double(subs(speed, tt, 0:dt:(2*pi)));

	arclen = cumsum(speed_eval * dt);
	segment = arclen(end) / npieces;
	t = [0];
	goal = segment;
	for i=1:length(arclen)
		if arclen(i) >= goal
			t = [t dt*(i-1)];
			goal = goal + segment;
		end
	end
	if length(t) < npts
        t = [t 2*pi];
        assert(length(t) == npts);
    end
		



	%t = 0:(2*pi/npieces):(2*pi);
	%assert(length(t) == npts);
	tshift = t - pi/2;
	xpos = (cos(tshift).^3 +2*cos(tshift))/3;
	ypos = sin(tshift) .* cos(tshift);
	%xvel = -sin(tshift);
	%yvel = cos(2*tshift);

	% mission with only position constraints
	x = nan(3, npts, 4);

	x(:,1,:) = 0;
	x(:,npts,:) = 0;
	for i=2:(npts-1)
		x(1,i,:) = [xpos(i) ypos(i) 0 0];
		%x(2,i,:) = [xvel(i) yvel(i) 0 0];
	end

	EXTRA_TIME_ACCEL = 0.5;
	t(2:end) = t(2:end) + EXTRA_TIME_ACCEL;
	t(end) = t(end) + EXTRA_TIME_ACCEL;
	[soln, free] = pp_waypoints(t, degree, continuity, x, 'monomial');
	fprintf('%d free dimensions\n', size(free, 2));

	pp = polyvec2pp(t, dim, soln);
    knot = t;
end
