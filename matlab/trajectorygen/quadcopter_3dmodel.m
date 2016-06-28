function group = quadcopter_3dmodel(arm)
% construct 3d model of a quadcopter with the given arm length
% (distance between diametrically opposed prop driveshafts)
	arm_wide = arm / 12;
	arm_thick = arm / 30;
	prop_thick = arm / 50;
	prop = arm / 4;
	arrow = arm / 2;
	arrow_thick = arrow / 15;

	[tprop, vprop] = disc(prop, prop_thick, 32);
	vprop = shift(vprop, [0 0 (arm_thick + prop_thick) / 2]);
	[txarm, vxarm] = box(arm + arm_wide, arm_wide, arm_thick);
	[tyarm, vyarm] = box(arm_wide, arm + arm_wide, 0.9 * arm_thick); % stop z-fighting

	[tzcyl, vzcyl] = disc(arrow_thick, arrow, 32);
	vzcyl = shift(vzcyl, [0 0 arrow/2]);
	[tzcone, vzcone] = cone(1.8 * arrow_thick, 3.5 * arrow_thick, 32);
	vzcone = shift(vzcone, [0 0 arrow]);
	[tzarrow, vzarrow] = mesh_concat(tzcyl, vzcyl, tzcone, vzcone);
    
    red   = [0.9 0.2 0.3];
    green = [0.2 0.7 0.3];
    blue  = [0.4 0.6 1.0];
	light = @(c) min(1.3 * c, 1);
	dark  = @(c) 0.7 * c;
    
	group = hgtransform;
	trisurf2(txarm, vxarm, 'Parent', group, 'FaceColor', red); 
	trisurf2(tyarm, vyarm, 'Parent', group, 'FaceColor', green);
	trisurf2(tprop, shift(vprop, [ arm/2 0 0]), 'Parent', group, 'FaceColor', light(red));
	trisurf2(tprop, shift(vprop, [-arm/2 0 0]), 'Parent', group, 'FaceColor', dark(red));
	trisurf2(tprop, shift(vprop, [0  arm/2 0]), 'Parent', group, 'FaceColor', light(green));
	trisurf2(tprop, shift(vprop, [0 -arm/2 0]), 'Parent', group, 'FaceColor', dark(green));
	trisurf2(tzarrow, vzarrow, 'Parent', group, 'FaceColor', blue); 
end

function h = trisurf2(t, v, varargin)
	h = trisurf(t, v(:,1), v(:,2), v(:,3),...
        'EdgeColor', 'None',...
        'AmbientStrength', 0.6, 'SpecularStrength', 0.3,...
        varargin{1:end});
end

function [t, v] = mesh_concat(varargin)
	if mod(nargin, 2) ~= 0
		error('number of args must be a multiple of 2');
	end
	t = [];
	v = [];
	for i=1:2:nargin
		t = [t; varargin{i} + size(v, 1)];
		v = [v; varargin{i+1}];
	end
end

function vv = shift(v, s)
	if size(s, 1) ~= 1
		s = s';
	end
	vv = bsxfun(@plus, v, s);
end

function [t, v] = cone(r, h, n)
% make a cone sitting on the xy plane pointing z+ with radius r, height h, n sides
	[t, v] = circle(r, n);
	v = [v; [0 0 h]];
	t1 = 2:n;
	t2 = t1 + 1;
	t3 = n+2 * ones(n-1, 1);
	t = [t; [t1' t2' t3]; [n+1 2 n+2]];
end

function [t, v] = circle(r, n)
% make a circle on the xy plane with radius r and n sides
	step = 2 * pi / n;
	theta = step * (0:(n-1));
	v = [r * cos(theta') r * sin(theta') zeros(n,1)];
	v = [0 0 0; v];
	t = [ones(n-1,1) (2:(n))' (3:(n+1))'];
	t = [t; 1 n+1 2];
end

function [t, v] = disc(r, h, n)
% make a disc centered on the xy plane with radius r, height h, and n sides
	step = 2 * pi / n;
	theta = step * (0:(n-1));
	vtop = [r * cos(theta') r * sin(theta') h/2 * ones(n,1)];
	vbottom = vtop;
	vbottom(:,3) = -h/2;
	v = [0 0 h/2; vtop; 0 0 -h/2; vbottom];
	ttop = [ones(n-1,1) (2:(n))' (3:(n+1))'];
	ttop = [ttop; 1 n+1 2];
	tbottom = n + 1 + ttop;
	v1 = 2:n;
	v2 = v1 + 1;
	v3 = v1 + n + 1;
	v4 = v3 + 1;
	tside = [v1' v2' v3'; v3' v2' v4'];
	tside = [tside; [n+1 2 n+1+n+1]; [n+1+n+1 2+n+1 2]];
	t = [ttop; tbottom; tside];
end

function [t, v] = cube()
% make an axis-aligned unit cube centered on the origin with side length 1
	vtop = [1 1 1; 1 -1 1; -1 -1 1; -1 1 1];
	vbottom = [vtop(:,1) vtop(:,2) -vtop(:,3)];
	v = 0.5 * [vtop; vbottom];
	t = [1 2 3; 1 3 4; 1 2 5; 2 6 5; 2 3 6; 3 7 6; 3 4 8; 3 8 7; 4 1 8; 1 5 8; 5 6 7; 5 7 8];
end

function [t, v] = box(x, y, z)
% make an axis-aligned box with the given dimensions
	[t, v] = cube();
	v = v * diag([x y z]);
end
