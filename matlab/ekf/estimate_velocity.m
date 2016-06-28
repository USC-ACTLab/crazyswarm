function v = estimate_velocity(positions, dt)
	len = size(positions, 2);
	t = (-len):(-1);
    t = dt * t;
	v = zeros(3, 1);
	for i=1:3
		p = polyfit(t, positions(i,:), 1);
        pd = polyder(p);
		v(i) = polyval(pd, 0);
	end
end
