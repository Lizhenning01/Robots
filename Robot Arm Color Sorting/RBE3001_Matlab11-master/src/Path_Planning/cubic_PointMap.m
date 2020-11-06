function points = cubic_PointMap(p1,p2)
%p1 is 1x3 xyz of starting point
%p2 is 1x3 xyz of final point

points =[pointMap(8,0,1,0,0,p1(1),p2(1)),pointMap(8,0,1,0,0,p1(2),p2(2)),pointMap(8,0,1,0,0,p1(3),p2(3))];
% points 5x3 of the cubically interpolated points
end

