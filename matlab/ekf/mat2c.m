function fname = mat2c(var, name)
	fname = [name '.h'];
	fd = fopen(fname, 'wt');
	[nr, nc] = size(var);
	decl = ['float ' name '[%d][%d] = {'];
	fprintf(fd, decl, nr, nc);
	for r=1:nr
		fprintf(fd, '\n\t{ ');
		fprintf(fd, '%.9g,', var(r,:));
		fprintf(fd, '},');
	end
	fprintf(fd, '\n};\n');
	fclose(fd);
end
