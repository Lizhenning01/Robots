function [imDetectedDisk, robotFramePose, diskDia] = findObjs(imOrig,cam,cameraParams,color)
% FINDOBJS implements a sequence of image processing steps to detect
% any objects of interest that may be present in an RGB image.
%
% Note: this function contains several un-implemented sections - it only
% provides a skeleton that you can use as reference in the development of
% your image processing pipeline. Feel free to edit as needed (or, feel
% free to toss it away and implement your own function).
%
%   Usage
%   -----
%   [IMDETECTEDOBJS, ROBOTFRAMEPOSE] = findObjs(IMORIG, TCHECKER2ROBOT, TCAM2CHECKER, CAMERAPARAMS)
%
%   Inputs
%   ------
%   IMORIG - an RGB image showing the robot's workspace (capture from a CAM
%   object).
%
%   TCHECKER2ROBOT - the homogeneous transformation matrix between the
%   checkered board and the reference frame at the base of the robot.
%
%   TCAM2CHECKER - the homogeneous transformation matrix between the camera
%   reference frame and the checkered board (you can calculate this using
%   the GETCAMTOCHECKERBOARD function, provided separately).
%
%   CAMERAPARAMS - an object containing the camera's intrinsic and
%   extrinsic parameters, as returned by MATLAB's camera calibration app.
%
%   Outputs
%   -------
%   Ideally, this function should return:
%   IMDETECTEDOBJS - a binarized image showing the location of the
%   segmented objects of interest.
%   
%   ROBOTFRAMEPOSE - the coordinates of the objects expressed in the robot's
%   reference frame
%
%   Authors
%   -------
%   Nathaniel Dennler  <nsdennler@wpi.edu>
%   Sean O'Neil        <stoneil@wpi.edu> 
%   Loris Fichera      <lfichera@wpi.edu>
%
%   Latest Revision
%   ---------------
%   2/12/2019


%%  1. First things first - undistort the image using the camera parameters
[im, ~] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');

%%  2. Segment the image to find the objects of interest.

%  [Your image processing code goes here]

% find the desired color
if(color == 'disk')
%     new_im = createBlackMask(imOrig);
    new_im = createBlackMaskLAB(imOrig);
elseif (color == 'yell')
    new_im = createYellowMask(imOrig); 
elseif (color == 'blue')
    new_im = createBlueMask(imOrig);
elseif(color == 'gree')
%     new_im = createGreenMask(imOrig);
    new_im = createGreenMaskLAB(imOrig);
elseif(color == 'duck')
%     new_im = createGreenMask(imOrig);
    new_im = createDuckMasker(imOrig);
else
    error('not valid color');
end


% 
% figure;
% imshow(new_im);

% find the holder ball 

if(color == 'disk')
    [center,radius] = imfindcircles(new_im, [28 150]);
else
    [center,radius] = imfindcircles(new_im, [15 30]);
end

% find the disk
% 


% disp(center);

%new_im = edge(new_im);



numCircle = size(center);
c_global = zeros(numCircle(1),4,'single');
robotFramePose = c_global;

% You can easily convert image pixel coordinates to 3D coordinates (expressed in the
% checkerboard reference frame) using the following transformations:\
for i = 1:numCircle(1)
    c_global (i,:) = transpose(getBaseToCam(cam, cameraParams,center(i,1:2)));
    if(color == 'disk')
        robotFramePose(i,1) = c_global(i,1)+10;
    elseif (color == 'yell')
        robotFramePose(i,1) = c_global(i,1)+25;
    elseif (color == 'blue')
        robotFramePose(i,1) = c_global(i,1)+25;
    elseif(color == 'gree')
        robotFramePose(i,1) = c_global(i,1)+25;
    elseif(color == 'duck')
        robotFramePose(i,1) = c_global(i,1)+15;
    else
        error('not valid color');
    end
    robotFramePose(i,2) =c_global(i,2); % 2cm off set in x
end
    

imDetectedDisk = numCircle(1);
diskDia = radius+10;
% see https://www.mathworks.com/help/vision/ref/cameraparameters.pointstoworld.html
% for details on the expected dimensions for YOUR_PIXEL_VALUES)
end