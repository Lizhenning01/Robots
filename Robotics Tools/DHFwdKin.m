function [T,Tn] = DHFwdKin(d,th,r,a)
%Takes arrays of each DH parameter and calculates all intermediate
% transformation matrices. 
% Returns 4x4xn dimensional T with all intermediate transformations and T0n.
% Angles in radians.

T = sym(zeros(4,4,size(d,2)));
for i=1:1:(size(d,2))
    T(:,:,i) = dh_link(d(i), th(i), r(i), a(i));
end

T0N = T(:,:,1);
for i=2:1:(size(T,3))
    T0N = T0N*T(:,:,i);
end
Tn=simplify(T0N);

end

