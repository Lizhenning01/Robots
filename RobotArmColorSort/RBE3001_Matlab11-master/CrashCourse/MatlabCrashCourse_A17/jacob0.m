function J = jacob0(q)
% takes a 3x1 q of the joint variables(degrees)
% returns the jacobian matrix J 6x3

    l1 = 135; %mm
    l2 = 175; %mm
    l3 = 169.28; %mm
%     T0 = [1,0,0,0;
%           0,1,0,0;
%           0,0,1,0;
%           0,0,0,1];
    T01 = fwkintrans(l1,deg2rad(q(1)),0,pi/2);
    T02 = T01 * fwkintrans(0,deg2rad(q(2)),l2,0);
    T03 = T02 * fwkintrans(0,deg2rad(q(3))+pi/2,-l3,0);
%     T01 = fwkintrans(l1,q(1),0,90);
%     T02 = T01 * fwkintrans(0,q(2),l2,0);
%     T03 = T02 * fwkintrans(0,q(3)+90,-l3,0);
    pe = T03(1:3,4);
    
    J = [cross([0;0;1],(pe-[0;0;0])),   cross(T01(1:3,3),(pe-T01(1:3,4))),  cross(T02(1:3,3),(pe-T02(1:3,4)));
        [0;0;1],                        T01(1:3,3),                         T02(1:3,3)                        ];
    Jp = J(1:3,:);
    
    if(det(Jp) < 100)
       annotation('textbox',[.25 .4 .5 .2],'String','APPROACHING SINGULARITY!!!!!!!','color','red','EdgeColor','none','fontweight','bold','FontSize',24);
       error('APPROACHING SINGULARITY!!!!!!!');
    end
end