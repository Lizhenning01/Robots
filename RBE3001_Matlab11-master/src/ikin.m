function joints_ang = ikin(x,y,z)
    L1 = 135;
    L2 = 175;
    L3 = 169.28; 
    
    inWorkspace = isInWorkspace(x,y,z);
    
    if (inWorkspace == 1)

        joints_rad = zeros(1,3,'single');

        joints_rad(1) = atan2(y,x);

        new_y = z-L1;
        new_x = sqrt(y^2+x^2);
        
        beta = acos((L2^2 + (y^2+x^2) + new_y^2 - L3^2)/(2*L2*sqrt(y^2+x^2 + new_y^2)));

        joints_rad(2) = beta + atan2(new_y,new_x);
        joints_rad(3) = -acos(-((L2^2 + L3^2 -(y^2+x^2 +new_y^2))/(2*L2*L3)))+3.1415/2;
        
%        joints_ang = joints_rad;
        joints_ang = rad2deg(joints_rad);
    else
        error('point out of bound');
    end
    % joint angles are in deg

end