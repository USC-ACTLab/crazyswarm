function ps = polyscale(p, s)
% scale the polynomial p so it has the same value at s*t that it had at t before
	p = p(:)'; % ensure row
	degree = length(p) - 1;
	pows = s .^ (degree:-1:0);
	ps = p ./ pows;
end
