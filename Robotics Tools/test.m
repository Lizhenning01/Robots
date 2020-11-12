%test bench or whatever
%
midterm_sol;

%%%%%%%%%%%%%%%%%%%%%
% test function [T,T0n] = DHFwdKin(d,th,r,a)
% syms th1 th2 th3 th4 th5 th6 L;
% thetas = [th1, th2, th3, th4, th5, th6];
% d  = [L + th1 0 0 0 th4+2*L 0 3*L];
% th = [0 th2 th3+pi -pi/2 th5+pi pi/2 th6];
% r  = [0 0 L L L -L 0];
% % a  = [0 0 -pi/2 -pi/2 pi pi/2 0];
% 
% syms q1 q2 q3 q4 q5 q6 L
% thetas = [q1 q2 q3 q4 q5 q6];
% d  = [L + q1 0 0 0 q4+2*L 0 3*L];
% th = [0 q2 q3+pi -pi/2 q5+pi pi/2 q6];
% r  = [0 0 L L L -L 0];
% a  = [0 0 -pi/2 -pi/2 pi pi/2 0];
% 
% [Tdh,T0ndh] = DHFwdKin(d,th,r,a);
% 
% solution_home = round(subs(T06,[q1,q2,q3,q4,q5,q6,L],[0,0,0,0,0,0,100]))
% T_dh_home = round(subs(T0ndh,[q1,q2,q3,q4,q5,q6,L],[0,0,0,0,0,0,100]))
% T_dh_home_pos_correct = isequal(solution_home,T_dh_home)
% 
% %%%%%%%%%%%%%%%%%%%%%
% % test function [T,T0n] = PoEFwdKin(M, Si, thetalist)
% syms q1 q2 q3 q4 q5 q6 L
% thetas = [q1 q2 q3 q4 q5 q6];
% 
M = [0 1  0 -3*L
     1 0  0 -L
     0 0 -1 -2*L
     0 0  0  1];
%  
% Si = [0 0  0  0 1  0
%       0 0 -1  0 0  0
%       0 1  0  0 0 -1
%       0 0  L -1 0  L
%       0 0  0  0 L -3*L
%       1 0  L  0 0  0];
%   
% T0nes = PoEFwdKin(M, Si, thetas);
% T_es_correct = isequal(T0nes,T06_sol)

%%%%%%%%%%%%%%%%%%%%%
% test function [T,T0n] = wqPoEFwdKin(wi, qi, thetalist, jointlist)
syms q1 q2 q3 q4 q5 q6 L p r
thetas = [q1 q2 q3 q4 q5 q6];

joints = [p r r p r r];

wi = [0 0  0  -1   1    0 
      0 0 -1   0   0    0 
      1 1  0   0   0   -1];
  
qi = [0 0 -L -2*L -3*L  -3*L
      0 0  0  0    0    -L
      L L  L  2*L  L    -2*L];
  
Si = GetSi(wi,qi,joints);

M = [0 1  0 -3*L
     1 0  0 -L
     0 0 -1 -2*L
     0 0  0  1];
 
T0nes = PoEFwdKin(M, Si, thetas);
% T_eswq_correct = isequal(T0nes,T06_sol)

[ES,T0nes2] = wqPoEFwdKin(wi, qi, thetas, joints);
isequal(T0nes,T0nes2)
%%%%%%%%%%%%%%%%%%%%%
% test function J = Jacobian(T,T0n,thetas,joints)
syms p r;
joints = [p r r p r r];
% Jdh = Jacobian(Tdh,T0ndh,thetas,joints);
Jes = Jacobian(Tes,T0nes,thetas,joints);

% Jdh_home = round(subs(Jdh,[q1 q2 q3 q4 q5 q6 L],[0 0 0 0 0 0 100]))
Jes_home = round(subs(Jes,[q1 q2 q3 q4 q5 q6 L],[0 0 0 0 0 0 100]))
