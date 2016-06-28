function trajectory_viz_piecewise(pp, npts, arm, mass, loop)
    
    if ~exist('loop', 'var')
        loop = false;
    end
%arm is in meters 
%mass is in kg

    [xyz, vel, acc, rotmtx, quat, rpy, omega, t] =...
        trajectory_eval_piecewise(pp, mass, npts);

    %figure();
    %plot3(xyz(1,:),xyz(2,:),xyz(3,:),'g');
    %grid on;
    %hold on;
    
    render_traj(xyz, rotmtx, npts, arm, loop);

end
