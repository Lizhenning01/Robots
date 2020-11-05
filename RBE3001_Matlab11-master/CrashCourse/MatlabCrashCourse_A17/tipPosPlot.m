function index = tipPosPlot(curPos,x_pos,y_pos,z_pos, ind)
            %tip position plot
            shoulder_ang = -enc2deg(curPos(1));
            elbow_ang = enc2deg(curPos(4));
            wrist_ang = enc2deg(curPos(7));
            tip = fwkin3001(shoulder_ang,elbow_ang,wrist_ang);
%             disp("tip");
%             disp(tip);
           
            x_pos.XData = [x_pos.XData ind];
            x_pos.YData = [x_pos.YData tip(1)];
            y_pos.XData = [y_pos.XData ind];
            y_pos.YData = [y_pos.YData tip(2)];
            z_pos.XData = [z_pos.XData ind];
            z_pos.YData = [z_pos.YData tip(3)];
            
            index = ind + 1;
end


