function pp = polyvec2pp(knot, dim, coeff)
% converts an k-piece, degree-d, dim-dimensional piecewise polynomial
% from a concatenated (d*k*n) column vector
% to a pp struct usable by Matlab's spline functions.
	pieces = length(knot) - 1;
	degree = length(coeff) / (pieces * dim);
	coeff = reshape(coeff, degree, pieces, dim);
	coeff = permute(coeff, [3 2 1]);
	pp = mkpp(knot, coeff, dim);
end
