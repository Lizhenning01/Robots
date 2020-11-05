function points = pointMap(ptNum,t0,tf,v0,vf,q0,qf)
    ceft = trajectoryGen(t0,tf,v0,vf,q0,qf);
    points = zeros(ptNum,1,'single');
    
    % put the coefficients into the cubic polynomial
    function cur_q = q(t , c)
        cur_q = c(1)+c(2)*t + c(3)*t^2 + c(4)*t^3; 
    end
    
    t_int = (tf-t0)/ptNum;
    
    for inc = 1:ptNum
%        points(inc) = deg2Enc(q((t0+t_int*inc), ceft));
         points(inc) = q((t0+t_int*inc), ceft);
    end
    
    % points are in encoder values
end