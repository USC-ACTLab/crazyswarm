function [soln, free] = pp_waypoints(t, d, c, x, varargin)
% finds the whole space of multidimensional piecewise polynomials that satisfy start/end constraints.
%
% t: times of waypoints, monotonically increasing
% d: polynomial degree
% c: continuity constraint. all derivatives through c are cts
% x: (1+n_derivatives) * n_points * n_dimensions array of constraints
%    e.g. x(1,1,1) is the position of the first waypoint in dimension 1
%         x(8,3,4) is the second derivative of the 8th waypoint in dimension 4
%    if a value in `x` is set to NaN:
%      - if the continity constraint c applies to that derivative, continuity is enforced
%      - otherwise, nothing happens (but this case should be rare in practice --
%        typically the continuity constraint is the same or greater order
%        than any of the waypoints' specified derivatives)
%
% Solution is a column vector containing all the pieces' coefficients 
% concatenated together by time, then by dimension.
% A polynomial of the form soln + free * v, for any v, is a solution.
% Use polyvec2pp to convert into a Matlab piecewise polynomial struct.
	
	dim = size(x, 3);
	free = cell(dim, 1);
	soln = cell(dim, 1);
	for i=1:dim
		[soln{i}, free{i}] = pp_waypoints_1d(t, d, c, x(:,:,i), varargin{:});
	end
	free = blkdiag(free{:});
	soln = cat(1, soln{:});
end

function [soln, free] = pp_waypoints_1d(t, d, c, x, varargin)
    dd = d+1; % total number of coeffs in the polynomial
	USE_PRECONDITION = true;
	USE_CHEBYSHEV = ~any(strcmpi(varargin, 'monomial'));
    USE_TIME_SCALE = true || USE_CHEBYSHEV; 

	if USE_CHEBYSHEV && length(uniquetol(diff(t),0.0001)) > 1
		error('current Chebyshev basis implementation requires uniform time intervals');
	end

	% scale time to help with matrix condition
    if USE_TIME_SCALE
		knot_intervals = diff(t);
        time_scale = 1 / max(knot_intervals);
        t = time_scale * t;
    end

	n_pieces = length(t) - 1;
	assert(size(x,2) == n_pieces + 1);

	maxderiv = size(x, 1) - 1;

	% later on, we will interpret NaNs as instruction to obey continuity constraint
	if maxderiv < c
		x = [x; nan(c - maxderiv, n_pieces + 1)];
		maxderiv = c;
	end

	b = [];
	A = [];

	% gives the indices for the coefficients of a specific polynomial piece
	function ind = p_idx(p)
		ind = (p - 1) .* dd + (1:dd);
	end
	% sanity check
	last_piece_ind = p_idx(n_pieces);
	assert(last_piece_ind (end) == n_pieces * dd);
    
	neqns = 0;

	% initial
	for deriv=0:maxderiv
		val = x(deriv+1,1);
		if ~isnan(val)
			b = [b val];
			A(p_idx(1),neqns+1) = time_vec(0, d, deriv);
			neqns = neqns + 1;
		end
	end

	% internal
	for pt=2:n_pieces
		for deriv=0:maxderiv
			val = x(deriv+1,pt);
			if ~isnan(val)
				% fixed value constraint
				b = [b val val];
				A(p_idx(pt-1),neqns+1) = time_vec(t(pt)-t(pt-1), d, deriv);
				A(  p_idx(pt),neqns+2) = time_vec(            0, d, deriv);
				neqns = neqns + 2;
			elseif deriv <= c
				% continuity constraint
				b = [b 0];
				A(p_idx(pt-1),neqns+1) = time_vec(t(pt)-t(pt-1), d, deriv);
				A(  p_idx(pt),neqns+1) = -time_vec(           0, d, deriv);
				neqns = neqns + 1;
			end
		end
	end

	% final
	for deriv=0:maxderiv
		val = x(deriv+1,end);
		if ~isnan(val)
			b = [b val];
			A(p_idx(n_pieces),neqns+1) = time_vec(t(end)-t(end-1), d, deriv);
			neqns = neqns + 1;
		end
	end

	if neqns > n_pieces * dd
		error('overconstrained problem');
	elseif neqns == n_pieces * dd
		warning('exactly contrained problem, no homogeneous solutions');
	end

	if USE_PRECONDITION
        conditioner = rowcond(A);
        A_precond = conditioner * A;
		% fprintf('precond(A): %f\n', cond(A_precond));
		condition_gain = cond(A) ./ cond(A_precond);
		if condition_gain > 10
			disp('using precondition');
			soln = conditioner * (b * pinv(A_precond))';
		else
			soln = (b * pinv(A))';
		end
		free = null(A');
	elseif USE_CHEBYSHEV
		chebs = flipud(chebpolys(d, 'shift'));
		repcheb = kron(eye(n_pieces), chebs);
		A_cheb = repcheb * A;
        condi = rowcond(A_cheb);
		%fprintf('cheb(A) cond: %f\n', cond(A_cheb));
        %b = condi * b;
        A_cheb_precond = condi * A_cheb;
		soln = repcheb' * condi * (b * pinv(A_cheb_precond))';
		free = repcheb' * null(A_cheb');
	else
		soln = (b * pinv(A))';
		free = null(A');
	end

	if USE_TIME_SCALE
		soln = unscale(soln, n_pieces, time_scale);
		for c=1:size(free,2)
			free(:,c) = unscale(free(:,c), n_pieces, time_scale);
		end
	end
    % NOTE: b/A also solves the problem.
	% it gives a sparser solution but b * pinv(A) has the smallest possible norm.
	% in my test, b/A had really harsh discontinuities in the first
	% noncontinuous derivative, whereas b * pinv(A) was almost smooth.

	% I don't think we really need this LQ decomposition
    %[free,q] = lq(null(A'));
end

function c = rowcond(A)
    row_maxes = max(abs(A), [], 2);
    c = diag(1.0 ./ row_maxes);
end

function ps = unscale(p, npieces, scale)
	p = reshape(p, [], npieces);
	for i=1:npieces
		p(:,i) = polyscale(p(:,i), 1.0 / scale);
	end
	ps = reshape(p, length(p(:)), 1);
end

function T = time_vec(time, degree, deriv)
	M = polydiffcfs(degree)';
	T = M^deriv * polyvander(time, degree)';
end
