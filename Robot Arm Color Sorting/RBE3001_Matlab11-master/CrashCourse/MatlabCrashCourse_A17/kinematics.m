% Code example for RBE3001 MATLAB crash course - Fall 2017
% 
% kinematic
%
% kinematic returns forward and inverse kinematic solutions for a simple 2
% degree of freedom robot arm. The robot parameters are passed to the
% function as an input.
%
% Syntax
%   y = kinematics(R,u,mode)
%
% Description
%
%   y = kinematics(R,u,mode) computes the kinematics solution of the
%   robot arm R based on the requested mode.
%   The mode input can either be 'forward' or 'inverse'; R is must be
%   an structure with fields l1 and l2; and u is the input point. 
%       mode='forward': 
%           - u must be a vector that contains the corresponding values of 
%             q1 and q2 as its first and second indices.
%           - The returned output y will be a 2 by 2 matrix of the form:
%                   y = [x1 y1;
%                        x2 y2];
%             where [x1, y1] indicate the Cartesian coordinates of the end
%             of the first link and [x2, y2] represent the Cartesian
%             coordinates of the end-effector.
%
%       mode='inverse': 
%           - u must be a vector that contains the corresponding values of 
%             x and y Cartesian coordinates as its first and second indices.
%           - The returned output y will be a 2 by 2 matrix of the form:
%                   y = [q1_1 q2_1;
%                        q1_2 q2_2];
%             where [q1_1, q2_1] are the corresponding joint angles for
%             solution 1 and [q1_2, q2_2] are the joint angles for solution
%             2 of the inverse kinematics.
%
% Author: Siamak G. Faal
%         sghorbanifaal@wpi.edu
%         http://users.wpi.edu/~sghorbanifaal/index.html
% Date: August 24, 2017


% Initialize the file as a function
function y = kinematics(R,u,mode)

% Detect the desired kinematic form
switch mode
    case 'forward' % If the user is asking for the forward kinematics
        y = R.l1*[cos(u(1)), sin(u(1))];
        y(2,:) = y + R.l2*[cos(u(1)+u(2)), sin(u(1)+u(2))];
        
    case 'inverse' % If the user is asking for the inverse kinematics
        Q2 = acos( (u(1)^2 + u(2)^2 - R.l1^2 - R.l2^2)/(2*R.l1*R.l2) );
        y(:,2) = [Q2;-Q2];
        for i=1:2
            a = R.l1 + R.l2*cos(y(i,2));
            b = R.l2*sin(y(i,2));
            r = 1/(a^2 + b^2)*[a b; -b a]*[u(1);u(2)];
            y(i,1) = atan2( r(2), r(1) );
        end
    otherwise % If the mode input niether 'forward' nor 'inverse'
        error('invalid mode input. mode can be "forward" or "inverse"');
end
        
end

