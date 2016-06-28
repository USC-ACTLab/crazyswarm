function c = chebpolys(degree, varargin)
% compute all the chebyshev polynomials up to given degree.
% returns a (degree + 1) square matrix by default, zero-padded
% so each row is a chebyshev polynomial suitable for use in polyval.
% if argument usecell is set to 'cell', returns a cell array
% of not-zero-padded row vectors instead, also suitable for polyval.
	tomtx = ~any(strcmp(varargin, 'cell'));
	shift = any(strcmp(varargin, 'shift'));
	x = [2 0];
	order = degree + 1;
	c = cell(order, 1);
	c{1} = [1];
	c{2} = [1 0];
	for i=3:order
		c{i} = polysum(conv(x, c{i-1}), -1 * c{i-2});
	end
	if shift
		xf = @(p) polytransform(p, [-1 1], [0 1]);
		c = cellfun(xf, c, 'UniformOutput', false);
	end
	if tomtx
		padder = @(p) zpad(p, order);
		pc = cellfun(padder, c, 'UniformOutput', false);
		c = cat(1, pc{:});
	end
end

function ps = polysum(p1, p2)
	len = max(length(p1), length(p2));
	ps = zpad(p1, len) + zpad(p2, len);
end

function zx = zpad(x, len)
	nz = len - length(x);
	zx = [zeros(1,nz) x];
end
