function p = minsnap(knot, dim, soln, free)
	function c = cost(x)
		pvec = soln + free * x;
		pp = polyvec2pp(knot, dim, pvec);
		snap = fnder(pp, 4);
		c = sum(intsqr(snap));
	end

	nfree = size(free, 2);
	x0 = zeros(nfree, 1);
	xopt = fminunc(@cost, x0);

	p = soln + free * xopt;
end

function s = intsqr(pp)
    sqr = fncmb(pp, '*', pp);
    intsqr = fnint(sqr);
    s = fnval(intsqr, pp.breaks(end)) - fnval(intsqr, pp.breaks(1));
end

function D = diffmtx(degree)
    D = diag(1:degree, 1);
end

function I = intmtx(degree)
    I = diag(1 ./ (1:(degree+1)), -1);
end