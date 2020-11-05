% makes a 3d stick figure model of 3 arm links
% @param q the 3x1 matrix of joint angles
function stickModel(q,q_vel)
    % getting all the joint xyz positions
    allJoints = fwkin_all(q(1),q(2),q(3));
    
    % separating the joint xyz positions into individual joints
    shoulder = [0, 0, 0];
    elbow = allJoints(:,1);
    wrist = allJoints(:,2);
    tip = allJoints(:,3);
    
    p_vel = fwkin_diff(q,q_vel);
    J = jacob0(q);
    Jp = J(1:3,:);
    
%     disp("tip");
%     disp(tip);
   
    % put all vectors in same plot
    % make the vectors of the links
    cla
    scatter3(shoulder(1),shoulder(2),shoulder(3));
            hold on

    plot3([shoulder(1), elbow(1)], [shoulder(2), elbow(2)], [shoulder(3), elbow(3)]);
    scatter3(elbow(1),elbow(2),elbow(3));
    plot3([elbow(1), wrist(1)], [elbow(2), wrist(2)], [elbow(3), wrist(3)]);
    scatter3(wrist(1),wrist(2),wrist(3));
    plot3([wrist(1), tip(1)], [wrist(2), tip(2)], [wrist(3), tip(3)]);
    scatter3(tip(1),tip(2),tip(3));
        
        %velocity vector quiver plot
    quiver3(tip(1),tip(2),tip(3),p_vel(1),p_vel(2),p_vel(3));
    plot_ellipse(Jp*transpose(Jp),[tip(1),tip(2),tip(3)]);

    xlim([-100 500]);
    ylim([-300 300]);
    zlim([-100 600]);
    grid on
    %hold off
end