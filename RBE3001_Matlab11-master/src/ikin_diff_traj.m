function qH = ikin_diff_traj(p_desired, q_initial)
    %path planning using differential kinematics
    %takes degrees for q_initial
    % Initialize qH as an empty matrix
    qH = [];
    q0_deg = q_initial;
    d = 100000;
    while (d > 0.5)
        q_deg = ikin_diff(p_desired, q0_deg);

        % Change initial to the current joint angles
        q0_deg = q_deg;
        % Store path history in qH
        qH(end+1:end+1,:) = q_deg;

        d_xyz = transpose(fwkin3001(q0_deg(1),q0_deg(2),q0_deg(3)))-p_desired;
        d = sqrt(d_xyz(1)^2 + d_xyz(2)^2 + d_xyz(3)^2);
        % disp(d);
    end

end

