function poly2csv(pp, filename)
	[breaks, coefs, npieces, order, dim] = unmkpp(pp);
	assert(dim == 4);
	coefs = reshape(coefs, 4, npieces, order);

	fid = fopen(filename, 'w');
	format long;
	vars = {'x' 'y' 'z' 'yaw'};
	fprintf(fid, 'duration,');
	for d=1:4
		for i=1:order
			fprintf(fid, '%s^%d,', vars{d}, i-1);
		end
	end
	fprintf(fid, '\n');
	for piece=1:npieces
		duration = breaks(piece+1) - breaks(piece);
		fprintf(fid, '%f,', duration);
		for d=1:4
			fprintf(fid, '%f,', flipud(squeeze(coefs(d,piece,:))));
        end
        fprintf(fid, '\n');
	end
end
