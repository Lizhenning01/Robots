function packet = goToPoint(point)
% point is (x y z) 
packet = zeros(15,1,'single');

buff = deg2Enc(ikin(point(1), point(2), point(3)));

packet(1) = -buff(1);
packet(4) = buff(2);
packet(7) = buff(3);

end

