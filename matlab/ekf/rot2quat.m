function quat = rot2quat(m)

if ndims(m) == 3
    quat = zeros(4, 1, size(m,3));
    for i=1:size(m,3)
        quat_temp = rot2quat_temp(m(:,:,i));
        quat(:,:,i) = quat_temp;        
    end
else
    quat = rot2quat_temp(m);
end


function quat = rot2quat_temp(m)

    tr = m(1,1) + m(2,2) + m(3,3);

    if tr > 0 
      S = sqrt(tr+1.0) * 2; % S=4*qw 
      qw = 0.25 * S;
      qx = (m(3,2) - m(2,3)) / S;
      qy = (m(1,3) - m(3,1)) / S; 
      qz = (m(2,1) - m(1,2)) / S; 
    elseif (m(1,1) > m(2,2)) && (m(1,1) > m(3,3))
      S = sqrt(1.0 + m(1,1) - m(2,2) - m(3,3)) * 2; % S=4*qx 
      qw = (m(3,2) - m(2,3)) / S;
      qx = 0.25 * S;
      qy = (m(1,2) + m(2,1)) / S; 
      qz = (m(1,3) + m(3,1)) / S; 
    elseif (m(2,2) > m(3,3)) 
      S = sqrt(1.0 + m(2,2) - m(1,1) - m(3,3)) * 2; % S=4*qy
      qw = (m(1,3) - m(3,1)) / S;
      qx = (m(1,2) + m(2,1)) / S; 
      qy = 0.25 * S;
      qz = (m(2,3) + m(3,2)) / S; 
    else
      S = sqrt(1.0 + m(3,3) - m(1,1) - m(2,2)) * 2; % S=4*qz
      qw = (m(2,1) - m(1,2)) / S;
      qx = (m(1,3) + m(3,1)) / S;
      qy = (m(2,3) + m(3,2)) / S;
      qz = 0.25 * S;
    end

    quat = [qx, qy, qz, qw]';
