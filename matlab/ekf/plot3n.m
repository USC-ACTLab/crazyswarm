function h = plot3n(x, varargin)
    sz = size(x);
    assert(length(sz) == 2);
    if sz(1) == 3 || sz(1) == 4
        h = plot3(x(1,:), x(2,:), x(3,:), varargin{:});
    elseif sz(2) == 3 || sz(2) == 4
        h = plot3(x(:,1), x(:,2), x(:,3), varargin{:});
    else
        error('argument x must be 3xN or Nx3');
    end
    xlabel('x');
    ylabel('y');
    zlabel('z');
    axis square;
end

