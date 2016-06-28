function n = normc(x)
	mag = sqrt(sum(x.^2, 1));
	n = bsxfun(@rdivide, x, mag);
end
