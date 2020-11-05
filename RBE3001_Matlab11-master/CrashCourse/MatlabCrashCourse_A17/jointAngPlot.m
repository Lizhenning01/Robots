function index = jointAngPlot(curPos,jointGraph0,jointGraph1,jointGraph2, ind)
    shoulder_ang = -curPos(1)*0.0879;
    elbow_ang = curPos(4)*0.0879;
    wrist_ang = curPos(7)*0.0879;

    jointGraph0.XData = [jointGraph0.XData ind];
    jointGraph0.YData = [jointGraph0.YData shoulder_ang];
    jointGraph1.XData = [jointGraph1.XData ind];
    jointGraph1.YData = [jointGraph1.YData elbow_ang];
    jointGraph2.XData = [jointGraph2.XData ind];
    jointGraph2.YData = [jointGraph2.YData wrist_ang];

    index = ind + 1;
end


