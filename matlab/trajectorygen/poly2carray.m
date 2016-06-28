function poly2carray(p, name, filename)
	fid = fopen(filename, 'w');
	format long;
	ncoeff = size(p, 2);
	fprintf(fid, 'float %s[4][%d] = {\n', name, ncoeff);
	for i=1:4
		fprintf(fid, '\t{');
		fprintf(fid, '%f, ', p(i,:));
		fprintf(fid, '},\n');
	end
	fprintf(fid, '};');
end
