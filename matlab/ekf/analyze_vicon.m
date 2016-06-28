function analyze_vicon(pos)
	dp = diff(pos, 1, 2);
	flat = all(dp == 0, 1);
	changes = 1 + find(~flat);
	if changes(1) ~= 1
		changes = [1 changes];
    end
    if changes(end) ~= size(pos, 2)
        changes = [changes size(pos, 2)];
    end

	deltas = diff(changes);
	unique_deltas = unique(deltas)
    
    hist(deltas,max(unique_deltas)-min(unique_deltas)+1);
end
