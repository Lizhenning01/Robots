function Si = GetSi(wi, qi, jointlist)
%Takes arrays of omega, q and a matrix of their joint types
%Returns a 6xi series of screw axis
%omega: axis of rotation for rotational OR axis of translation for prismatic
%qi: location of joint in reference to base frame
%jointlist: syms 'p' or 'r' to denote joint type of each column
syms p r;

 Si = sym(zeros(6,size(wi,2)));
 for i=1:1:size(wi,2)
     if isequal(jointlist(i), r)
        Si(1:3,i) = wi(:,i);
        Si(4:6,i) = cross(-wi(:,i),qi(:,i));
     end
     if isequal(jointlist(i), p)
         Si(1:3,i) = [0; 0; 0];
         Si(4:6,i) = wi(:,i);
     end
 end
 
end

