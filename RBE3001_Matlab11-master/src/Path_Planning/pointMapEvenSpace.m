function pmap = pointMapEvenSpace(p1,p2)
%takes two points (x,y,z) and returns ten setpoints evenly spaced between them

ptNum = 10;
pmap = zeros(ptNum,3);


x_int = (p2(1)-p1(1))/ptNum;
y_int = (p2(2)-p1(2))/ptNum;
z_int = (p2(3)-p1(3))/ptNum;

for h=1:ptNum
    pmap(h,1)=p1(1)+x_int(1,1)*(h);
    pmap(h,2)=p1(2)+y_int(1,1)*(h);
    pmap(h,3)=p1(3)+z_int(1,1)*(h);
    
end


end

