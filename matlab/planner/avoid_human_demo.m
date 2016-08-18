function main()

    mex -v GCC='/usr/bin/gcc-4.7' CFLAGS="\$CFLAGS -std=c99" -I../../crazyflie-firmware/src/modules/interface -I../../crazyflie-firmware/src/modules/src avoidtarget_mex.c;
	
    HZ = 30;
	FRAMETIME = 1/HZ;

	tic;

	clf;
	p = avoidtarget_mex(double(toc));
	x = p(1,:);
	y = p(2,:);
	h = scatter(x, y);
	xlim([x(1) - 1, x(end) + 1])
	ylim([y(1) - 1, y(end) + 1])
	h.XDataSource = 'x';
	h.YDataSource = 'y';

	mouse = get(gca, 'CurrentPoint');


	function tick(t, eventdata)
		if ~ishandle(h)
			stop(t);
            delete(t);
            return;
		end

		time = double(toc);
		mouse2 = [mouse(1) mouse(3)]
		p = avoidtarget_mex(time, mouse2);
		x = p(1,:);
		y = p(2,:);

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
