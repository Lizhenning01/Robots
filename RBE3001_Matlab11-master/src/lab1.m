%%
% RBE3001 Lines 15-37 perform necessary library initializations. You can
% skip reading to line 38.
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
try
    SET_POS = 01;       % we will be talking to server ID 01 on the Nucleo
    PID_CONFIG = 02;
    CALIB = 9;
    STATUS = 8;
    DEBUG = false;      % enables/disables debug prints
    
    empty_packet = zeros(15, 1, 'single');
    packet = zeros(15, 1, 'single');
    
    
    
    % 3 points that make a triangle across 3 axes
        viaPts = [
            deg2Enc(ikin(175,0,0)); % zero pos
            deg2Enc(ikin(256, 2, 140));
            deg2Enc(ikin(117.3, 82.63,0));
            deg2Enc(ikin(175,0,0))];  % zero pos
    
    
%     % cubic_inp trangle
%     viaPts = [
%         cubic_PointMap([175,0,0],[256,2,140]);
%         cubic_PointMap([256,2,140],[117.3, 82.63,0]);
%         cubic_PointMap([117.3, 82.63,0],[175,0,0])];
%  for loop= 1:3 
%     viaPts = rad2deg(simulate());
%     
    numPts = size(viaPts);
    %     disp(viaPts);
    
    % %     %     variables/settings to start live plot of encoder values
    %     figure
    %     hold on
    %     grid on
    %     %         xlim([-50 450]);
    %     %         ylim([-250 250]);
    %     ind = 0;
    %     shoulder_ang = 0;
    %     elbow_ang = 0;
    %     wrist_ang = 0;
    %     jointGraph0 = line(ind,shoulder_ang,'color','green','LineWidth',2);
    %     jointGraph1 = line(ind,elbow_ang,'color','blue','LineWidth',2);
    %     jointGraph2 = line(ind,wrist_ang, 'color','red','LineWidth',2);
    %
    for k = [1:25:numPts(1),numPts(1)] %puts set point positions in to 'packet', sends the points to the robit and captures the 'returnPacket'
        % %        tic
        point = viaPts(k,:);
        packet(1) = deg2Enc(-point(1));
        packet(4) = deg2Enc(point(2));
        packet(7) = deg2Enc(point(3));
        %pp.write(SET_POS, goToPoint(point));
  
        pp.write(SET_POS, packet);
        pause(0.003); % Minimum amount of time required between write and read
        returnPacket = pp.read(SET_POS);
        
                        disp(packet);
                        disp(returnPacket);
        % %        toc
        %          pause(1); %pause to allow robot to move to desired setpoint -
        % %         currently using the 'for' loop below for the delay
        %
%         obtain encoder values for plotting
                for h = 1:125
                    pp.write(STATUS, empty_packet);  % acquire encoder status
                    pause(0.003);    % Minimum amount of time required between write and read
                    curPos = pp.read(STATUS);
        %             %             %%joint angle plot(degrees)
        %             %             ind = jointAngPlot(curPos,jointGraph0,jointGraph1,jointGraph2,ind);
        %             %
        %             %             %%tip position plot
        %             ind = tipPosPlot(curPos,jointGraph0,jointGraph1,jointGraph2,ind);
        %             %
                                %%graph trace of tip position
                                        shoulder_ang = -curPos(1)*0.0015;
                                        elbow_ang = curPos(4)*0.0015;
                                        wrist_ang = curPos(7)*0.0015;
                                        tip = fwkin3001(shoulder_ang,elbow_ang,wrist_ang);
                                        plot(tip(1),tip(2),'x','color','red'); drawnow
        % %            toc
                end
        %         % stick animation
%         for h = 1:1
%             pp.write(STATUS, empty_packet); % acquire encoder status
%             pause(0.08); % Minimum amount of time required between write and read
%             curPos = pp.read(STATUS);
%             pause(0.05);
%             stickRender(curPos);
%             %disp("current");
%             %disp(curPos);
%         end
    end
%  end
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

xlabel('X [mm]'), ylabel('Y [mm]'),zlabel('Y [mm]');
title('Singularity Configuration #2');
%legend('x','y');
% legend('x','y','z');
%legend('j0 vel','j1 vel','j2 vel');

% Clear up memory upon termination
pp.shutdown()
