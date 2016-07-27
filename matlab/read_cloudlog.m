function [stamps, clouds] = read_cloudlog(path)
	f = fopen(path, 'r');
	stamps = [];
	clouds = {};
	while true
		stamp = fread(f, 1, 'uint32');
		if feof(f)
			break
		end
		stamps = [stamps stamp];
		count = fread(f, 1, 'uint32');
		cloud = fread(f, [3 count], 'float32');
		clouds{end+1} = cloud;
	end
end
