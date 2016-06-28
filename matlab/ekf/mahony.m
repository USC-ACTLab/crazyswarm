% Madgwick's implementation of Mayhony's AHRS algorithm.
% See: http:%www.x-io.co.uk/open-source-ahrs-with-x-imu
%
% Date         Author            Notes
% 29/09/2011 SOH Madgwick        Initial release
% 02/10/2011 SOH Madgwick    Optimised for reduced CPU load
function qs = mahony(qinit, omegas, accs, dt)

	N = size(omegas, 2);
	qs = zeros(4, N);

	TWO_KP_DEF = (2.0 * 0.4); % 2 * proportional gain
	TWO_KI_DEF = (2.0 * 0.001); % 2 * integral gain

	twoKp = TWO_KP_DEF;        % 2 * proportional gain (Kp)
	twoKi = TWO_KI_DEF;        % 2 * integral gain (Ki)
	integralFBx = 0.0;
	integralFBy = 0.0;
	integralFBz = 0.0;    % integral error terms scaled by Ki

	q0 = qinit(4);
	q1 = qinit(1);
	q2 = qinit(2);
	q3 = qinit(3);

	for i=1:N
		% Normalise accelerometer measurement
		acc = accs(:,i);
		acc = acc / norm(acc);
		ax = acc(1);
		ay = acc(2);
		az = acc(3);

		gx = omegas(1,i);
		gy = omegas(2,i);
		gz = omegas(3,i);

		% Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5 + q3 * q3;

		% Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		% Compute and apply integral feedback if enabled
		if twoKi > 0.0
			integralFBx = integralFBx + twoKi * halfex * dt;    % integral error scaled by Ki
			integralFBy = integralFBy + twoKi * halfey * dt;
			integralFBz = integralFBz + twoKi * halfez * dt;
			gx = 			gx + integralFBx;    % apply integral feedback
			gy = 			gy + integralFBy;
			gz = 			gz + integralFBz;
		else
			integralFBx = 0.0; % prevent integral windup
			integralFBy = 0.0;
			integralFBz = 0.0;
		end

		% Apply proportional feedback
		gx = 		gx + twoKp * halfex;
		gy = 		gy + twoKp * halfey;
		gz = 		gz + twoKp * halfez;

		% Integrate rate of change of quaternion
		gx = 		gx * (0.5 * dt);     % pre-multiply common factors
		gy = 		gy * (0.5 * dt);
		gz = 		gz * (0.5 * dt);
		qa = q0;
		qb = q1;
		qc = q2;
		q0 = 		q0 + (-qb * gx - qc * gy - q3 * gz);
		q1 = 		q1 + (qa * gx + qc * gz - q3 * gy);
		q2 = 		q2 + (qa * gy - qb * gz + q3 * gx);
		q3 = 		q3 + (qa * gz + qb * gy - qc * gx);

		% Normalise quaternion
		recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 = 		q0 * recipNorm;
		q1 = 		q1 * recipNorm;
		q2 = 		q2 * recipNorm;
		q3 = 		q3 * recipNorm;

		qs(:,i) = [q0 q1 q2 q3]';
	end

	qs = qs([2 3 4 1],:);
end
