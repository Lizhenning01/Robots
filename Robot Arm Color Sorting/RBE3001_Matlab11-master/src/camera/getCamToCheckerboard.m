function T_cam_to_checker = getCamToCheckerboard(cam, cameraParams)
% GETCAMTOCHECKERBOARD Calculates the homogeneous transformation matrix between the camera's reference
% frame and the reference frame of a checkerboard.
%   
%   Usage
%   -----
%   T = GETCAMTOCHECKERBOARD(CAM, CAMERAPARAMS)
%
%   Inputs
%   ------
%   CAM - a WEBCAM object. In MATLAB, instances of this class, offer an
%   interface to a digital imaging device connected to the computer.
%   To see a list of available imaging devices, type in the MATLAB prompt:
%   
%   >> webcamlist
%
%   In most cases, you will have a single imaging device connected to your 
%   computer. You can create a reference to the device using the WEBCAM
%   function:
%
%   >> cam = webcam();
%
%   CAMERAPARAMS - an object containing the camera's intrinsic and
%   extrinsic parameters, as returned by MATLAB's camera calibration app.
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

%%  1. Capture an image from the imaging device
    imOrig = snapshot(cam);

%%  2. Undistort the image
    [im, newOrigin] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');

%%  3. Detect the checkerboard in the image
    [imagePoints, boardSize] = detectCheckerboardPoints(im);

%%  4. Adjust the imagePoints so that they are expressed in the coordinate system
    % used in the original image, before it was undistorted.  This adjustment
    % makes it compatible with the cameraParameters object computed for the original image.
    imagePoints = imagePoints + newOrigin; % adds newOrigin to every row of imagePoints

%%  5. Compute the transformation between the checkerboard and the camera
    [R, t] = extrinsics(imagePoints, cameraParams.WorldPoints, cameraParams);
    
    T_cam_to_checker = [R t'; 0 0 0 1];
    
%%  6. Plotting
    axesPoints = worldToImage(cameraParams, R, t, [0 0 0; 0 50 0; 50 0 0]);
    
    x1 = [axesPoints(1,1) axesPoints(2,1)]';
    y1 = [axesPoints(1,2) axesPoints(2,2)]';

    im = insertText(im, [x1(2) y1(2)], 'Y Axis', 'TextColor', 'green', 'FontSize', 18);    
    x2 = [axesPoints(1,1) axesPoints(3,1)]';
    y2 = [axesPoints(1,2) axesPoints(3,2)]';
    
    im = insertText(im, axesPoints(3,1:2), 'X Axis', 'TextColor', 'red', 'FontSize', 18);
    
    imshow(im)
    title('Undistorted Image with checkerboard axes');

    line(x1, y1,'lineWidth', 5, 'Color', 'green' );
    line(x2, y2,'lineWidth', 5, 'Color', 'red');
end