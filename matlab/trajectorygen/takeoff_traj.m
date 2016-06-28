function [pp, vec] = takeoff_traj(helipad, radius)
	
	npts = 2;
	npieces = npts - 1;

    degree = 7;
    continuity = 4;
	dim = 4;

	x = zeros(3, npts, 4);
	x(1,1,:) = [0 0 1 0];
    x(1,2,:) = [0 0 0.05 0];

	knot = [0 2];

	[vec, free] = pp_waypoints(knot, degree, continuity, x);
	fprintf('%d free dimensions\n', size(free, 2));

	pp = polyvec2pp(knot, dim, vec);
end
