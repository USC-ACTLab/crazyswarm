function d = find_diverging_col(a, b)
	different = any(abs(a - b) > 0.000001); % any collapses rows
	d = find(different, 1);
end
