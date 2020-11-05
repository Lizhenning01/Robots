function calibration()
%calibration command: sets HomePosition and PID coefficients
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
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

pp = PacketProcessor(myHIDSimplePacketComs);
PID_CONFIG = 02;
CALIB = 9;
STATUS = 8;
packet = zeros(15, 1, 'single');
empty_packet = zeros(15, 1, 'single');

try
    %----------------------calibrate the PID------------------------
    Kpid = [.001, 0, 0; %joint 0
        .002, 0, 0.01; %joint 1
        .003, 0, 0]; %joint 2
    
    for k = 0:2
        joint = Kpid((k+1),:);
        packet(3*k+1) = joint(1);
        packet(3*k+2) = joint(2);
        packet(3*k+3) = joint(3);
    end
    pause(0.005);
    pp.write(PID_CONFIG, packet);
    
    %---------------------set home position-------------------------
    count = 0; % count number of times values taken for averaging
    homeBuff = zeros(15,1,'single');
    
    for h = 1:10
        pp.write(STATUS, empty_packet); % acquire encoder status
        pause(0.08); % Minimum amount of time required between write and read
        home_pos = pp.read(STATUS);
        %disp("home position");
        %disp(home_pos);
        
        if (home_pos(1) ~= 0)
            homeBuff = homeBuff + home_pos;
            count = count + 1;
        end
    end
    
    if (count ~= 0)
        average = homeBuff / count;
        pp.write(CALIB, average);
        pause(0.005);
    end
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
pp.shutdown()
end

