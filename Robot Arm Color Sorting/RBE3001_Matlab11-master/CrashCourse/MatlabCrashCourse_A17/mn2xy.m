% This function is used to convert an m/n pixel coordinate to an x/y
% position coordinate.  This function centers the x/y coordinate system
% at the robot's end-effector's home position, with +x pointing towards
% the camera, and +y following the RHR.
%
% In addition:  this function assumes that you have successfully
% run the provided "calibrate_camera.m" function prior to its use.
function [ outarr ] = mn2xy( m, n )
%% define calibration distance constants
tot_width_in_cm = (2.5*12);
tot_height_in_cm = (2.5*8);

%% read in data from xml
xml = xmlread('calibrations/pixels.xml');
xml_pixels = xml.getElementsByTagName('pixel');
pixels = zeros(5,2);
for i = 1:5
   pixels(i, :) = extract_ith_pixel(i-1, xml_pixels);
end

%% organizing data by row.  2nd col *should* be consistent
arm_pixels = pixels(1:2,:);
hole_pixel = pixels(3,:);
cam_pixels = pixels(4:5,:);

%% commonly re-used calibration parameter
cam_height_in_pix = mean(cam_pixels(:,2));
arm_height_in_pix = mean(arm_pixels(:,2));
arm_to_hole = hole_pixel(2) - arm_height_in_pix;
cam_to_hole = cam_height_in_pix - hole_pixel(2);
tot_height_in_pix = cam_height_in_pix - arm_height_in_pix;
cam_width_in_pix  = cam_pixels(end,1) - cam_pixels(1,1);
arm_width_in_pix  = arm_pixels(end,1) - arm_pixels(1,1);

%% calculate x using n
xdist = (n - hole_pixel(2));
if xdist < 0
x = ((tot_height_in_cm*.45)/(arm_to_hole))*(xdist)
else
x = ((tot_height_in_cm*.55)/(cam_to_hole))*(xdist)
end
%% calculate y using m and n
sf_cam = (tot_width_in_cm/cam_width_in_pix); %interpolate between
sf_arm = (tot_width_in_cm/arm_width_in_pix);
percentage = (cam_height_in_pix-n)/(tot_height_in_pix);
sf_cur = percentage * (sf_arm - sf_cam) + sf_cam;
y = sf_cur * (m - hole_pixel(1))
outarr = [x,y];
end

% burrows into xml object and rips out numbers
function [pix] = extract_ith_pixel(i, pixels)
    pix(1) = str2num(pixels.item(i).getElementsByTagName('m').item(0).getTextContent());
    pix(2) = str2num(pixels.item(i).getElementsByTagName('n').item(0).getTextContent());
end
