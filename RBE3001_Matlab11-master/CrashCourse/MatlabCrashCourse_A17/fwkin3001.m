function tip_pos = fwkin3001(q0,q1,q2)
    %takes deg

    l1 = 135; %mm
    l2 = 175; %mm
    l3 = 169.28; %mm
    
    T03 = fwkintrans(l1,deg2rad(q0),0,3.1415/2)*fwkintrans(0,deg2rad(q1),l2,0)*fwkintrans(0,deg2rad(q2)+3.1415/2,-l3,0);
    
    tip_pos = T03(1:3,4);
end