function main()
	HZ = 30;
	FRAMETIME = 1/HZ;
	STEP = 0.6;
	N = 7
	MAXSPEED = 3.0; % m/s
	MAXDISPLACE = 1.2 * STEP;
	steps = STEP * ((1:N) - mean(1:N));
	% "home" positions
	[hx, hy] = meshgrid(steps);
	hx = hx(:);
	hy = hy(:);
	% current positions
	x = hx;
	y = hy;
	% velocity
	vx = 0 * x;
	vy = 0 * y;

	clf;
	h = scatter(x, y);
	xlim([x(1) - 2*STEP, x(end) + 2*STEP])
	ylim([y(1) - 2*STEP, y(end) + 2*STEP])
	h.XDataSource = 'x';
	h.YDataSource = 'y';

	mouse = get(gca, 'CurrentPoint');

	function [gx, gy] = goal()
		dx = x - mouse(1);
		dy = y - mouse(3);
		r = sqrt(dx.^2 + dy.^2);
		dxunit = dx ./ r;
		dyunit = dy ./ r;

		force = 1.5 ./ (r + r.^2);
		displacement = min(force, MAXDISPLACE);
		gx = hx + displacement .* dxunit;
		gy = hy + displacement .* dyunit;
	end

	function tick(t, eventdata)
		tic;
		if ~ishandle(h)
			stop(t);
            delete(t);
            return;
		end

		[gx, gy] = goal();
		dx = gx - x;
		dy = gy - y;
		r = sqrt(dx.^2 + dy.^2);
		dxunit = dx ./ r;
		dyunit = dy ./ r;
		speed = min(4*r, MAXSPEED);
		%vx = 0.5*vx + speed .* dxunit;
		%vy = 0.5*vy + speed .* dyunit;
		x = x + FRAMETIME * speed .* dxunit;
		y = y + FRAMETIME * speed .* dyunit;

		refreshdata(h, 'caller');
	end


	function mousemove(object, eventdata)
		mouse = get(gca, 'CurrentPoint');
	end

	t = timer('Period', 1/30,... %period
		'ExecutionMode', 'fixedRate',... %{singleShot,fixedRate,fixedSpacing,fixedDelay}
		'BusyMode','drop',... %{drop, error, queue}
		'TasksToExecute',inf,...          
		'StartDelay',0,...
		'TimerFcn', @tick,...
		'StartFcn',[],...
		'StopFcn',[],...
		'ErrorFcn',[]);
	start(t);

	set(gcf, 'WindowButtonMotionFcn', @mousemove);
end
