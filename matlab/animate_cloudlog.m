function animate_cloudlog(path)
	[stamps, clouds] = read_cloudlog(path);
	[x, y, z] = decompose(clouds{1});
	h = scatter3(x, y, z, 'filled');
    axis equal;
	h.XDataSource = 'x';
	h.YDataSource = 'y';
	h.ZDataSource = 'z';
    EVERY_K = 1;
	for i=2:length(stamps)
        if mod(i, EVERY_K) ~= 0
            continue;
        end
        if size(clouds{i}, 2) == 0
            continue;
        end
		[x, y, z] = decompose(clouds{i});
		refreshdata(h, 'caller');
		drawnow;
        if mod(i, 100) == 0
            fprintf('i = %d\n', i);
        end
	end
end

function [x, y, z] = decompose(c)
    % throw away markers outside our Vicon area
    outside_arena = any(abs(c) > 4, 1);
    c(:,outside_arena) = [];
	x = c(1,:);
	y = c(2,:);
	z = c(3,:);
end
