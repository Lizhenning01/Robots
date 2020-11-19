function Tf = BuildTransform(omega,joint_pos)
%BUILDTRANSFORM takes joint orientation and position in reference to base
%frame and returns a transformation matrix for it
%IMPORTANT: BUSTED
%currently ignores all if statements for some stupid matlab reason


T(1:3,4) = cross(-omega,joint_pos);
omega = transpose(omega);
T(1:3,3) = omega;
T(4,1:4) = [0 0 0 1];

if isequal(omega, [1, 0, 0])
    T(1:3,1) = [0; 0; -1];
    T(1:3,2) = [0; 1; 0];
end
if  isequal(omega, [-1, 0, 0])
    T(1:3,1) = [0; 0; 1];
    T(1:3,2) = [0; 1; 0];
end
if isequal(omega, [0, 1, 0])
    T(1:3,1) = [1; 0; 0];
    T(1:3,2) = [0; 0; -1];
end
if isequal(omega, [0, -1, 0])
    T(1:3,1) = [1; 0; 0];
    T(1:3,2) = [0; 0; 1];
end
if isequal(omega, [0, 0, 1])
    T(1:3,1) = [1; 0; 0];
    T(1:3,2) = [0; 1; 0];
end
if isequal(omega, [0, 0, -1])
    T(1:3,1) = [1; 0; 0];
    T(1:3,2) = [0; -1; 0];
end

Tf = T;
end

