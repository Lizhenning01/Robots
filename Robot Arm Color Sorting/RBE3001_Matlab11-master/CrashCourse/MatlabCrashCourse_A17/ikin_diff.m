function q_desired = ikin_diff(p_desired, q_initial)
% simulate inverse differential kinematics with jacobian
% p_desired = [x,0,z]; q_desired = [0,q1,q2];
% for 2D simulation

q_cur = q_initial;

J = jacob0(q_cur);

q_delta = pinv(J(1:3,1:3)) * (transpose(p_desired)-fwkin3001(q_cur(1),q_cur(2),q_cur(3)));
q_desired = q_cur + transpose(q_delta);

end