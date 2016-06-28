function acc_imu = addgravityandrotateacc(acc, rotmtx)
	g = 9.81;
	acc(3,:) = acc(3,:) + g;
    N = size(acc,2);
    acc_imu = zeros(3,N);
    for i=1:N
        acc_imu(:,i) = rotmtx(:,:,i)' * acc(:,i);
    end
end
