function [ES,T0n,T_inc] = wqPoEFwdKin(wi, qi, M, thetalist, jointlist)
%Takes arrays of M, omega (orientation of joint) and q, (location of joint)
% 
% Returns 4x4xn dimensional T with all intermediate transformations and T0n.
% Angles in radians.
syms p r;

 ES = sym(zeros(4,4,size(wi,2)));
 I = eye(3);
 for i=1:1:max(size(thetalist))
     theta = thetalist(i);
     
     if isequal(jointlist(i), r)
         v = cross(-wi(:,i),qi(:,i));
         w = GetSkewOmega(wi(:,i));
     end
     if isequal(jointlist(i), p)
         w = GetSkewOmega([0; 0; 0]);
         v = wi(:,i);
     end
     
     R = I+(sin(theta)*w)+((1-cos(theta))*(w^2));
     V = (I*theta+(1-cos(theta))*w+(theta-sin(theta))*w^2)*v;
     ES(:,:,i) = [[R; 0 0 0], [V;1]];
 end
 
 T0n = ES(:,:,1);
 T_inc(:,:,1) = sym([eye(3), [0;0;0]; [0,0,0,1]]);
 for i=2:1:(size(ES,3))
     T0n = T0n*ES(:,:,i);
     Mn = BuildTransform(wi(:,i),qi(:,i));
     T_inc(:,:,i)=Mn;
     ES(:,:,i) = ES(:,:,i-1)*ES(:,:,i)*Mn;
 end
 
 T0n=simplify(T0n*M);
 
end

