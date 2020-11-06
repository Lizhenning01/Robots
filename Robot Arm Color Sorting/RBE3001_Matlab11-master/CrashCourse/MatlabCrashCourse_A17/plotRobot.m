% Code example for RBE3001 MATLAB crash course - Fall 2017
% 
% plotRobot
%
% Updates robot figure handle to show it at a configuration specified by
% the input u.
%
% Syntax
%   plotRobot(R,q)
%
% Description
%
%   plotRobot(R,q) updates the line handle in R (R.handle) to show the arm
%   at a configuration specified by the input q (a vector that hold the 
%   values of q1 and q2 as its first and second indices).
%
% Author: Siamak G. Faal
%         sghorbanifaal@wpi.edu
%         http://users.wpi.edu/~sghorbanifaal/index.html
% Date: August 24, 2017

% Initialize the file as a function
function plotRobot(R,q)

% Use kinematics function to calculate the link positions in the Cartesian
% coordinate system
y = fwkin3001(q(1),q(2),q(3));

% Change the x and y data in R.handle to the current configuarion (q)
set(R.handle,'XData',[0;y(:,1)],'ZData',[0;y(:,2)]); % TODO add z-data and remove y-data

