function rpy = quat2rpy(qs)

if ndims(qs) == 3
    rpy = zeros(3, 1, size(qs,3));
    for i=1:size(qs,3)
        rpy_temp = quat2rpy_temp(qs(:,:,i));
        rpy(:,:,i) = rpy_temp;        
    end
else
    rpy = quat2rpy_temp(qs);
end


function rpy = quat2rpy_temp(qs)
% convert quaternion to roll pitch yaw angles

rpy = [
    atan2(2*(qs(1,:).*qs(2,:)+qs(3,:).*qs(4,:)),1-2*(qs(2,:).^2+qs(3,:).^2));
    asin(2*(qs(1,:).*qs(3,:) - qs(4,:).*qs(2,:)));
    atan2(2*(qs(1,:).*qs(4,:)+qs(2,:).*qs(3,:)),1-2*(qs(3,:).^2+qs(4,:).^2));
];