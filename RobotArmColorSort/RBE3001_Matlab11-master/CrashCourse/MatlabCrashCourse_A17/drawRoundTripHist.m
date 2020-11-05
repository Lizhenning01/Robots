
function drawRoundTripHist()
%takes matrix of round-trip times from 'tic/toc' and draws a histogram with
%increments of .00005s

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
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

STATUS = 8;
empty_packet = zeros(15, 1, 'single');

close;
lap_time = zeros(1,500);

pp = PacketProcessor(myHIDSimplePacketComs);

for test = 1:500
    tic
    pp.write(STATUS, empty_packet); % acquire encoder status
    pause(0.003); % Minimum amount of time required between write and read
    sampleRead = pp.read(STATUS);
    lap_time(1,test) = toc;
end

hist(lap_time(1,2:500),0.003: 0.00005 :0.005);
%disp(lap_time);
xlabel('Round Trip Time (s)'), ylabel('Number of Trips');
title('Round Trip Time Histogram');

a=['Mean: ', num2str(mean(lap_time))];
b=['Standard deviation: ', num2str(std(lap_time))];
c=['Min: ', num2str(min(lap_time)), ' Max: ', num2str(max(lap_time))];
disp(a);
disp(b);
disp(c);

end

