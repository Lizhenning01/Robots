function packet = gripper(gripperPos)
%GRIPPER makes the servo move
% gripperPos 0 is for closed position, 1 is for open position

    if (gripperPos > 1 || gripperPos < 0)
        error("INVALID GRIPPER POSITION. STOP.");
    else
        packet = zeros(15, 1, 'single');
        packet(1) = gripperPos; % actuate gripper; 0 for close and 1 for open
    end


end

