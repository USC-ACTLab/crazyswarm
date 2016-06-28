function [p_out, v_out, a_out, q_out] = integrate(acc, gyro, pos, quat, dt)

p_out = NaN + pos;
q_out = NaN + quat;
v_out = NaN + pos;
a_out = NaN + pos;

N = size(acc, 2);

p = pos(:,1);
q = quat(:,1);
v = (pos(:,11) - pos(:,1)) / (10 * dt);

for i=1:N
    p_out(:,i) = p;
    q_out(:,i) = q;
    v_out(:,i) = v;
    
    omega = gyro(:,i); % no bias
    q = qnorm(quat_gyro_update(q, omega, dt));
    
    acc_imu = acc(:,i);
    acc_world = qvrot(qinv(q), acc_imu);
    acc_world(3) = acc_world(3) - 9.81;
    a_out(:,i) = acc_world;
    
    v = v + dt * acc_world;
    p = p + dt * v;
end

end


function qq = qinv(q)
    qq = [-q(1); -q(2); -q(3); q(4)];
end

function qq = qvrot(q, v)
    qq = 2 * dot(q(1:3), v) * q(1:3) + ...
         (q(4)^2 - dot(q(1:3), q(1:3))) * v + ...
         2 * q(4) * cross(q(1:3), v);
end

function qq = quat_gyro_update(q, omega, dt)
	r = (dt / 2) * omega(1);
	p = (dt / 2) * omega(2);
	y = (dt / 2) * omega(3);

    qq = zeros(4,1);
	qq(1) =    q(1) + y*q(2) - p*q(3) + r*q(4);
	qq(2) = -y*q(1) +   q(2) + r*q(3) + p*q(4);
	qq(3) =  p*q(1) - r*q(2) +   q(3) + y*q(4);
	qq(4) = -r*q(1) - p*q(2) - y*q(3) +   q(4);
end

function qq = qnorm(q)
    qq = q ./ norm(q);
end