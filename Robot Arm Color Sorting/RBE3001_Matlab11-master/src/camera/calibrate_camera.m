% This function is used to calibrate the mapping from camera pixel
% coordinates to x/y coordinates (on your board).  To use:
% 1) run function--observe snapshot image appear from your camera
% 2) click each of the 4 marked points in the snapshot, in order 1-4
% 3) press "enter" when you have clicked on each point
% 4) Done.  You'll notice that you've created a 'calibration_points.xml'
% file.  Now that this exists, you can successfully use "ij2xy(i,j)"

%% Instantiate hardware (turn on camera)
if ~exist('cam', 'var') % connect to webcam iff not connected
    cam = webcam();
    pause(1); % give the camera time to adjust to lighting
end

%% Request two points from user to define bounding box
img = snapshot(cam);
imshow(img);
[x, y] = getpts;
save_calibration_points(x,y);

function [] = save_calibration_points(x, y)
    % create xml object
    xml = com.mathworks.xml.XMLUtils.createDocument('pixels');
    root = xml.getDocumentElement();

    for i = 1:length(x) % add all points to doc
        point = xml.createElement('pixel');
        xNode = xml.createElement('m'); 
        yNode = xml.createElement('n');

        xNode.appendChild(xml.createTextNode(num2str(x(i))));
        yNode.appendChild(xml.createTextNode(num2str(y(i))));
        point.appendChild(xNode);
        point.appendChild(yNode);
        root.appendChild(point);
    end
    xmlwrite('calibrations/pixels.xml',xml);
end