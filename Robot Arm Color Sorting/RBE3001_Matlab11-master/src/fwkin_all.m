function joint_positions = fwkin_all(q0,q1,q2)
% ANGLES IN deg
    l1 = 135; %mm
    l2 = 175; %mm
    l3 = 169.28; %mm
    

    T01 = fwkintrans(l1,deg2rad(q0),0,3.1415/2);
    T02 = T01 * fwkintrans(0,deg2rad(q1),l2,0);
    T03 = T02 * fwkintrans(0,deg2rad(q2)+3.1415/2,-l3,0);
    
    joint_positions = [T01(1:3,4),T02(1:3,4),T03(1:3,4)];
end