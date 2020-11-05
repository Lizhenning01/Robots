% Auto-generated by cameraCalibrator app on 13-Feb-2019
%-------------------------------------------------------


% Define images to process
imageFileNames = {'/ifs/home/ashen/Documents/RBE3001_Matlab11/Image2.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image3.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image4.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image5.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image6.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image7.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image8.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image9.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image10.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image11.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image13.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image14.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image16.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image18.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image20.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image21.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image22.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image23.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image24.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image26.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image27.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image28.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image29.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image30.png',...
    '/ifs/home/ashen/Documents/RBE3001_Matlab11/Image31.png',...
    };

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates of the corners of the squares
squareSize = 12;  % in units of 'millimeters'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams);

% Visualize pattern locations
h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% For example, you can use the calibration data to remove effects of lens distortion.
undistortedImage = undistortImage(originalImage, cameraParams);

% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('MeasuringPlanarObjectsExample')
% showdemo('StructureFromMotionExample')
