function qH = simulate()
% Code example for RBE3001 MATLAB crash course - Fall 2017
%
% A simple animation of a 2 degree of freedom planar arm. Through this
% simulation, user can select as many desired points for the robot
% end-effector by left clicking on the figure. If the desired point falls
% within the workspace of the robot (indicated by dashed circles), the arm
% will move to the desired target. User can finish the execution of the
% simulation by right clicking on the figure. After the simulation, all the
% path points of the robot are saved in �history.csv� file.A simple
% animation of a 2 degree of freedom planar arm. Through this simulation,
% user can select as many desired points for the robot end-effector by
% left clicking on the figure. If the desired point falls within the
% workspace of the robot (indicated by dashed circles), the arm will move
% to the desired target. User can finish the execution of the simulation
% by right clicking on the figure. After the simulation, all the path
% points of the robot are saved in �history.csv� file.
%
%
% Author: Siamak G. Faal
%         sghorbanifaal@wpi.edu
%         http://users.wpi.edu/~sghorbanifaal/index.html
% Date: August 24, 2017


% Clear "Command Window", clear all the variables in "Workspace" and close
% all the figures.
clc; clear; close all;

% Creating �Robot� structure
Robot.l1 = 175;
Robot.l2 = 169.28;

% Defining workspace radiuses of the Robot
workspace(2) = Robot.l1 + Robot.l2;
workspace(1) = abs(Robot.l1 - Robot.l2);

% Initial conditions
q0_rad = [0 0 0];
q0_deg = rad2deg(q0_rad);


% Initializing figure
figure();           % Open (create) a figure environment
axes;               % Add an axes environment to the figure
hold on;            % Hold on to the objects in the axes
axis equal;         % Set the aspect ratios of the coordinates to be equal
box on; grid on;    % Put a box around axes and display the grid lines
% axis(300*[-1 1 -1 1]); % Set axes limits
xlim([0 270]);
ylim([-180 300]);
title({'Left click to set a point','Right click to exit'}); % Add title

% Plot the two dashed circles that indicate the workspace of the robot
% w = linspace(0,2*pi,100);
% for i=1:2
%     plot(workspace(i),workspace(i),'--k'); % TODO fix this
% end

% Convert the initial angles to the arm positions in Cartesian coordinate
% y0 = kinematics(Robot,q0,'forward');
% T02 = fwkintrans(Robot.l1,deg2rad(q0_deg(1)),0,3.1415/2)   *  fwkintrans(0,deg2rad(q0_deg(2)),Robot.l2,0);
% T03 = fwkin3001(q0_deg(1), q0_deg(2), q0_deg(3));
all_jointPos = fwkin_all(q0_deg(1), q0_deg(2), q0_deg(3));

% T12 = fwkintrans(0,deg2rad(q0_deg(2)),Robot.l1,0);
% T23 = fwkintrans(0,deg2rad(q0_deg(2))+3.1415/2,-Robot.l2,0);
y0 = [all_jointPos(1,2),all_jointPos(3,2);
    all_jointPos(1,3),all_jointPos(3,3)];

% Set "handle" field in "Robot" structure to be the handle to the arm plot
Robot.handle = plot([0;y0(:,1)],[135;y0(:,2)],'-o','color',[0 0.4 0.7],...
    'LineWidth',4,'MarkerSize',10,'MarkerFaceColor','c');

% Store the handle to the target point in "point"
point = plot(y0(2,1),y0(2,2),'rx','MarkerSize',15,'LineWidth',2);

% Initialize qH as an empty matrix
qH = [];

% Set the gains to compute inner product (used to define the distance
% between two set of joint angles)
G = [10 0; 0 1];
con = 1;

% Start an infinite loop
while(con)
    % Get a point with coordinates x, y and the mouse click number b
    [x,y,b] = ginput(1);
    
    % Compute the distance between the point and the origin
    %d = norm([x;y]);
    
    if(b > 1) % Check if user did a left click or not
        break; % If it is not a left click, break out of the "while" loop
        %     elseif( d > workspace(2) || d < workspace(1))
        %         continue; % If the point is not in the workspace, do not execute
        %                   % the rest and get a new point
    end
    
    % Use kinematics function to find the joint angles that will put the
    % end-effector in the target location
    %q = kinematics(Robot,[x y],'inverse');
    p_desired = [x,0,y];
    % Move the target point of the figure to x and y coordinates
    set(point,'XData',x,'YData',y);
    d = 1000000;
    
    while (d > 0.001)
        cla
        q_deg = ikin_diff(p_desired, q0_deg);
        q_rad = deg2rad(q_deg);
        % Divide the distance between current and desired joint angles into 100
        % steps
        q_buff_rad(:,1) = zeros(1,1,'single');
        q_buff_rad(:,2) = linspace(q0_rad(2),q_rad(2),1);
        q_buff_rad(:,3) = linspace(q0_rad(3),q_rad(3),1);
        
        
        % Display robot at each q_ value that ranges from the initial to the
        % desired joint angles
        for i=1:1
            q_cur_rad = q_buff_rad(i,:);
            q_cur_deg = rad2deg(q_cur_rad);
            
            all_jointPos = fwkin_all(0, q_cur_deg(2), q_cur_deg(3));
            
            % T12 = fwkintrans(0,deg2rad(q0_deg(2)),Robot.l1,0);
            % T23 = fwkintrans(0,deg2rad(q0_deg(2))+3.1415/2,-Robot.l2,0);
            y = [all_jointPos(1,2),all_jointPos(3,2);
                all_jointPos(1,3),all_jointPos(3,3)];

            
            % Set "handle" field in "Robot" structure to be the handle to the arm plot
            Robot.handle = plot([0;y(:,1)],[135;y(:,2)],'-o','color',[0 0.4 0.7],...
                'LineWidth',4,'MarkerSize',10,'MarkerFaceColor','c');
            
            %plotRobot(Robot,q_buff_rad(i,:));
            drawnow(); % Update the figure
        end
        % Change initial to the current joint angles
        q0_deg = q_deg;
        q0_rad = q_rad;
        % Store path history in qH
        qH(end+1:end+1,:) = q_cur_rad;
        
        d_xyz = transpose(fwkin3001(q0_deg(1),q0_deg(2),q0_deg(3)))-p_desired;
        d = sqrt(d_xyz(1)^2 + d_xyz(2)^2 + d_xyz(3)^2);
        % disp(d);
    end
    con = 0;
    disp("done");
    

                
%             
%             pp.write(SET_POS, goToPoint(point));
%             pause(0.003); % Minimum amount of time required between write and read
%             returnPacket = pp.read(SET_POS);
%             

end

% Write qH to "history.csv" file
csvwrite('history.csv',qH);
end