function J = Jacobian(T,T0n,omega,thetas,joints)
%Returns the Jacobian using 4x4xn collection of transformation matrices.
% Uses differential calculation method.
syms p r;
J = sym(zeros(max(size(thetas)),6));
for i=1:1:max(size(thetas))
    J(1:3,i) = diff(T0n(1:3,4),thetas(i));
    if isequal(joints(i),r)
%         J(4:6,i) = T(1:3,3,i);
        J(4:6,i) = omega(:,i);
    end
    if isequal(joints(i),p)
        J(4:6,i) = [0;0;0];
    end
end
J = vpa(J);
end

