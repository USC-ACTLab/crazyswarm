% plots the indices of the 2d points at the point locations.
% returns a graphics handle.

function h = idxplotw(x, y)

	xrange = max(x) - min(x);
	yrange = max(y) - min(y);
	xlim([min(x) - 0.2 * xrange, max(x) + 0.2 * xrange]);
	ylim([min(y) - 0.2 * yrange, max(y) + 0.2 * yrange]);

	for k=1:length(x)
		text(x(k), y(k), num2str(k));
	end
end

