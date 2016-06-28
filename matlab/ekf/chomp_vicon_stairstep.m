function p = fix_vicon_stairstep(pos)
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
	x = interp1(changes, pos(1,changes), all_t, 'spline');
	y = interp1(changes, pos(2,changes), all_t, 'spline');
	z = interp1(changes, pos(3,changes), all_t, 'spline');
	p = [x; y; z];
	assert(all(size(p) == size(pos)));
    assert(~any(isnan(p(:))));
end
