function render_traj(xyz, rotmtx, npts, arm, loop)
    clf;
    %hold off;
    %figure();
    q = quadcopter_3dmodel(arm);
    hold on;
	axis equal;
	camva(camva);
	light('Position', [-10 -5 20]);
	light('Position', [2 10 2]);
	light('Position', [0 0 -1]);
    %    trajectory_eval(A, mass, 0, 3, npts);
    bbox = [min(xyz,[],2)-arm, max(xyz,[],2)+arm]';
    bbox = bbox(:);
    
    plot3(xyz(1,:), xyz(2,:), xyz(3,:));
    axis(bbox);
    
    function once()
        m = eye(4);
        for i=1:npts
            m(1:3,1:3) = rotmtx(:,:,i);
            m(1:3,4) = xyz(:,i);
            set(q, 'Matrix', m);
            drawnow;
            %drawnow nocallbacks;
        end
    end

    if exist('loop', 'var')
        try
            while true
                once();
            end
        catch
            % swallow error from x-ing out of render window
        end
    else
        once();
    end
end