function stickRender(pos)

    % pos is a 15x1 matrix with encoder values

%     
%     shoulder_ang = -pos(1)*dcon;
%     elbow_ang = pos(4)*dcon;
%     wrist_ang = pos(7)*dcon;    
%     disp("angles");
%     disp(shoulder_ang);
%     disp(elbow_ang);
%     disp(wrist_ang);
    
    stickModel([(enc2deg(-pos(1)));(enc2deg(pos(4)));(enc2deg(pos(7)))],[(enc2deg(-pos(2)));(enc2deg(pos(5)));(enc2deg(pos(8)))]);

end

%implementation of this function:
%         % stick animation
%             for h = 1:100
%                 pp.write(STATUS, empty_packet); % acquire encoder status
%                 pause(0.08); % Minimum amount of time required between write and read
%                 curPos = pp.read(STATUS);
%                 pause(0.05);
%                 stickRender(curPos);
%             disp("current");
%             disp(curPos);
%             end