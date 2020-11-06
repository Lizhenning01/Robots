function p_vel = fwkin_diff(q,q_vel)
    % q is  3x1 vector of joint-space position(degrees)
    % q_vel is 3x1 vector of joint-space velocity (degree/sec)
    p_vel = jacob0(q)*q_vel;
    
end