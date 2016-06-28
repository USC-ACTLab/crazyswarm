function pt = polytransform(p, xold, xnew)
% transform the polynomial p so it takes on the same values
% on the interval xnew that it used to take on xold
	pstart0 = polyshift(p, -xold(1));
	scale = diff(xnew) / diff(xold);
	pscaled = polyscale(pstart0, scale);
	pt = polyshift(pscaled, xnew(1));
end
