%% RBE3001 Team 11 Final Project

clear
clear java
clear classes;
clear all;
vid = hex2dec('3742');
pid = hex2dec('0007');
disp (vid);
disp (pid);
javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
% version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

warning off;
close all; %gets rid of previous run's figures
% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs);
% disp("before try catch");


%%
try
    SET_POS = 01;       % we will be talking to server ID 01 on the Nucleo
    STATUS = 8;
    GRIPPER = 7;
    DEBUG = false;      % enables/disables debug prints
    
    empty_packet = zeros(15, 1, 'single');
    packet = zeros(15, 1, 'single');
    
    pp.write(GRIPPER, gripper(1));
    pause(0.2);
    pp.write(SET_POS, goToPoint([175,0, 50]));
    pause(0.5);
    
    
    
    % getting camera calibration data from saved matlab script
    load('calibrationSession2.mat');
    camParam = calibrationSession.CameraParameters;
    % connecting to webcam
    cam = webcam();
    
    for i = 1:2
        sortObjects('gree', cam, camParam, pp, STATUS,GRIPPER,SET_POS);
        sortObjects('blue', cam, camParam, pp, STATUS,GRIPPER,SET_POS);
        sortObjects('yell', cam, camParam, pp, STATUS,GRIPPER,SET_POS);
    end
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()


%% move to the general area of the ball function
function moveToBall(ball, pp, STATUS,GRIPPER, SET_POS)
% ball is the (x y) global cordinate of the ball object

pp.write(GRIPPER, gripper(1));
pause(0.2);

pp.write(SET_POS, goToPoint([ball(1)-20,ball(2),50]));
pause(0.5);

end


%% grabbing function
function grab(ball, pp,STATUS,GRIPPER, SET_POS)
% ball is the (x y) global cordinate of the ball object.
%pp.write(GRIPPER, gripper(1));
%pause(0.2);
pp.write(SET_POS, goToPoint([ball(1),ball(2),50]));
pause(0.8);

path = ikin_diff_traj([ball(1),ball(2),-35],ikin(ball(1),ball(2),20));
%     pause(0.1);

    numPts = size(path);
    
    for i = 1:(numPts(1)-200)
        point = path(i,:);
        packet(1) = deg2Enc(-point(1));
        packet(4) = deg2Enc(point(2));
        packet(7) = deg2Enc(point(3));
        %pp.write(SET_POS, goToPoint(point));
  
        pp.write(SET_POS, packet);        
        pause(0.02);
    end
    for i = (numPts(1)-200):numPts(1)
        point = path(i,:);
        packet(1) = deg2Enc(-point(1));
        packet(4) = deg2Enc(point(2));
        packet(7) = deg2Enc(point(3));
        %pp.write(SET_POS, goToPoint(point));
  
        pp.write(SET_POS, packet);        
        pause(0.005);
    end

% 
% path = cubic_PointMap([ball(1),ball(2),20], [ball(1),ball(2),-35]);
% 
% for i = 1:8
%     pp.write(SET_POS, goToPoint(path(i,:)));
%     pause(0.3);
% end

pp.write(GRIPPER, gripper(0));
pause(0.2);
pp.write(SET_POS, goToPoint([ball(1),ball(2),80]));
pause(0.8);
pp.write(SET_POS, goToPoint([175,0,80]));
pause(0.8);
end

%% finds the disk size of a given handle position
function my_disk_dia = findDiskSize(ball_pos,num_disk, disk_pos, disk_dia)
my_obj = 0;
min_dist = 10000000;
for i = 1:num_disk
    d = sqrt((ball_pos(1)-disk_pos(i,1)).^2+(ball_pos(2)-disk_pos(i,2)).^2);
    
    if d < min_dist
        my_obj = i;
        min_dist = d;
    end
end

my_disk_dia = disk_dia(my_obj);

end

%% pick and sort routine
function sortObjects(color, cam, camParam, pp, STATUS,GRIPPER,SET_POS)
noObject = 0;
objectFound = 0;

cur_ball = [0, 0];
size = 0;


while noObject <=2 && objectFound <=2
    cur_im = snapshot(cam);
    [num_ball, ball_pos, ball_size] = findObjs(cur_im,cam,camParam,color);
    if (num_ball ~= 1)
        disp('cant find object or multiple same-color object present');
        noObject = noObject +1;
        objectFound = 0;
    else
        noObject = 0;
        
%         %checks for disk size
%         [num_disk, disk_pos, disk_dia] = findObjs(cur_im,cam,camParam,'disk');
%         my_disk_dia = findDiskSize(ball_pos,num_disk, disk_pos, disk_dia);
%         disp(my_disk_dia);
%         if(my_disk_dia > 58) %simple threshold for determining big or small
%             %             disp("big disk");
%             size = -1;
%         else
%             %             disp("small disk");
%             size = 1;
%         end
        
        
        moveToBall(ball_pos,pp,STATUS,GRIPPER,SET_POS); %move to location above the ball
        
        
        d = sqrt((cur_ball(1)-ball_pos(1,1)).^2+(cur_ball(2)-ball_pos(1,2)).^2);
        
        if  d < 2
            objectFound = objectFound + 1;
        else
            objectFound = 0;
            
        end
        
        cur_ball = ball_pos(1,1:2);
        %             disp(objectFound);
        %             disp(noObject);
    end
end



if objectFound > 0
    if (color == 'blue')
        imshow('Blue.png');
    elseif (color == 'gree')
        imshow('Green.png');
    elseif (color == 'yell')
        imshow('Yellow.png');
    end 
    
    cur_im = snapshot(cam);
        %checks for disk size
    [num_disk, disk_pos, disk_dia] = findObjs(cur_im,cam,camParam,'disk');
    my_disk_dia = findDiskSize(ball_pos,num_disk, disk_pos, disk_dia);
    disp(my_disk_dia);
    if(my_disk_dia > 58) %simple threshold for determining big or small
        %             disp("big disk");
        size = -1;
    else
        %             disp("small disk");
        size = 1;
    end
    
    %move arm down and grab ball
    grab(cur_ball, pp, STATUS, GRIPPER, SET_POS);
    
    % sort
    sort(color, size, pp, STATUS, GRIPPER, SET_POS);
    
    %         path = cubic_PointMap([175,0,80], [175,(size*270),50]);
    %         pause(0.1);
    %
    %         for i = 1:8
    %             pp.write(SET_POS, goToPoint(path(i,:)));
    %             pause(0.5);
    %         end
    %             %open the gripper before shutdown
    %         pp.write(GRIPPER, gripper(1));
    %         pause(0.2);
    %         pp.write(SET_POS, goToPoint([175,0, 40]));
    %         pause(0.5);
end

end

function sort(color,my_size, pp, STATUS, GRIPPER, SET_POS)

    disp(color);
    
    
    path = ikin_diff_traj([175,(my_size*250),50],ikin(175,0,80));
    pause(0.1);

    numPts = size(path);
    
    for i = 1:10:numPts(1)
        point = path(i,:);
        packet(1) = deg2Enc(-point(1));
        packet(4) = deg2Enc(point(2));
        packet(7) = deg2Enc(point(3));
        %pp.write(SET_POS, goToPoint(point));
  
        pp.write(SET_POS, packet);        
        pause(0.08);
    end

%     pp.write(SET_POS,goToPoint([175,(my_size*270),50]));
%     pause(0.5);
    
    %open the gripper before shutdown
    pp.write(GRIPPER, gripper(1));
    pause(0.2);
    pp.write(SET_POS, goToPoint([175,0, 40]));
    pause(0.5);
end
