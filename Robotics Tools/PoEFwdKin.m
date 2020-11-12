function [T0n] = PoEFwdKin(M, Si,thetalist)
%Takes arrays of M and Si and calculates T0n
% Returns 4x4 T0n.
% Angles in radians.

 ES = sym(zeros(4,4,size(Si,2)));
 I = eye(3);
 for i=1:1:max(size(thetalist))
     theta = thetalist(i);
     S = Si(:,i);
     w = GetSkewOmega(S);
     R = I+(sin(theta)*w)+((1-cos(theta))*(w^2));
     V = (I*theta+(1-cos(theta))*w+(theta-sin(theta))*w^2)*S(4:6);
     ES(:,:,i) = [[R; 0 0 0], [V;1]];
 end
 
 T0n = ES(:,:,1);
 for i=2:1:(size(ES,3))
     T0n = T0n*ES(:,:,i);
 end
 
 T0n=simplify(T0n*M);
 
end

