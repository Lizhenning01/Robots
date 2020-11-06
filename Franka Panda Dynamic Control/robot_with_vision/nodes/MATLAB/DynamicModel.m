%% DYNAMICAL MODEL OF FRANKA EMIKA PANDA ROBOT
% Foltan 8/5/2020
% MATLAB script for calculating Forward Kinematics, Inverse Kinematics, and
% Dynamical Model for Franke Emika Panda Robot
% System requirements: ROS Toolbox

%% Load Robot Models
% Uncomment when it is necessary to recalculate the symbolic forward
% kinematics, inverse kinematics or dyanmical model

%configureMatlab(); quit;

% When no symbolic recalculation is necessary:
%load('Panda Config');
%folderpath = ('~/catkin_ws/src/Robot_Dynamics/');
%rosgenmsg(folderpath);
%addpath('/home/parallels/catkin_ws/src/Robot_Dynamics/matlab_gen/msggen')

rosinit('http://parallels-Parallels-Virtual-Platform:11311/');

block_points = rossubscriber('/block_points', @blocksFound);

% ideally set up the publisher and pass it in to the publish_joint function somehow.
% Current instantiates every iteration of publish_joint
%joint_state_pub = rospublisher('/joint_states', 'sensor_msgs/JointState');

%% Perform Desired Calculations

%% ROS Functions
function blocksFound(src, msg)
x_points = msg.XPoints;
y_points = msg.YPoints;
z_points = msg.ZPoints;
points = cell(1, length(x_points));
for i = 1:length(x_points)
points{i} = [str2double(x_points{i}) str2double(y_points{i}) str2double(z_points{i})];
end

[jointspaceconfig, tauconfig] = plan_trajectory(points{1}, [0.5, 0.5, 0]);

vel = cell(1,7);
vel{1} = 0; vel{2} = 0; vel{3} = 0; vel{4} = 0; vel{5} = 0; vel{6} = 0; vel{7} = 0;
for i = 1:length(jointspaceconfig)
    disp("publishing");
    publish_joint(jointspaceconfig{1}, [0;0;0;0;0;0;0], tauconfig{1});
    pause(5);
end
end


%% Trajectory Functions
function [jointspaceconfig, tauconfig] = plan_trajectory(startpos, endpos)
load('Panda Config');


liftpos = [startpos(1);startpos(2);startpos(3)+0.3];
tstart = 0;
tend = 5;
tlift = tend/2;

syms t
% Create Task Space Trajectories
x(t) = piecewiseCubicTraj(tstart, tlift, tend, startpos(1), liftpos(1), endpos(1));
y(t) = piecewiseCubicTraj(tstart, tlift, tend, startpos(2), liftpos(2), endpos(2));
z(t) = piecewiseCubicTraj(tstart, tlift, tend, startpos(3), liftpos(3), endpos(3));

% Choose 5 locations along the trajectory
k = 5;
chosentimes = linspace(0,tend,k);

taskspaceconfig = cell(k, 1);
for i = 1:k
    t = chosentimes(i);
    x_value = double(subs(x));
    y_value = double(subs(y));
    z_value = double(subs(z));
    taskspaceconfig{i} = [x_value; y_value; z_value];
    
end


% Find corresponding joint space configurations
jointspaceconfig = cell(k, 1);
for i = 1:k
    disp("Finding jointspaceconfig" + i);
    %jointspaceconfig{i} = num_IK(J, T, [0;0;0;0;0;0;0], taskspaceconfig{i});
    jointspaceconfig{i} = [0; 0; 0; 0; 0; 0; 0];
end

% Calculate Efforts
tauconfig = cell(k,1);
for i = 1:k
    tauconfig{i} = calcTau(tau, jointspaceconfig{i}, [0;0;0;0;0;0;0],[0;0;0;0;0;0;0]);
end

end

function p =  piecewiseCubicTraj(tstart, tlift, tend, startpos, liftpos, endpos)
syms t
p(t) = piecewise(t<=tlift, cubicTraj(tstart, tlift, startpos, liftpos), t<=tend, cubicTraj(tlift, tend, liftpos, endpos));

end

function t = cubicTraj(tstart, tend, startpos, endpos)
syms t
m = [1 tstart tstart^2 tstart^3;
    1 tend tend^2 tend^3;
    0 1 2*tstart 3*tstart^2;
    0 1 2*tend 3*tend^2];

syms a0 a1 a2 a3
S = solve(m*[a0; a1; a2; a3] == [startpos; endpos; 0;0]);
a = double([S.a0; S.a1; S.a2; S.a3]);

t(t) = a(1) + a(2)*t + a(3)*t^2 + a(4)*t^3;

end

%% Numeric Model Functions
function tauNum = calcTau(tau, angle, velocity, acceleration)
%{ 
    Takes - tau - symbolic Dyanmical Model
            angle - 7x1 vector of q values
            velocity - 7x1 vector of qdot values
            acceleration - 7x1 vector of qdotdot values
    Returns - tauNum - 7x1 vector of joint taus
%}

% Assign Symbolic Variables & Calculate Tau
q1 = angle(1); q2 = angle(2); q3 = angle(3); q4 = angle(4); q5 = angle(5); q6 = angle(6); q7 = angle(7);
q1d = velocity(1); q2d = velocity(2); q3d = velocity(3); q4d = velocity(4); q5d = velocity(5); q6d = velocity(6); q7d = velocity(7);
q1dd = acceleration(1); q2dd = acceleration(2); q3dd = acceleration(3); q4dd = acceleration(4); q5dd = acceleration(5); q6dd = acceleration(6); q7dd = acceleration(7);

tauNum = double(subs(tau));
end

function FKNum = calcFK(T, angles)
%{
    Takes - T - symbolic cell array of Transformation matrices wrt {0}
            angles - 7x1 vector of q values
    Returns - FKNum - 3x1 vector - the position of the end effector
%}

%Assign Symbolic Variables & Calculate pe
q1 = angles(1); q2 = angles(2); q3 = angles(3); q4 = angles(4); q5 = angles(5); q6 = angles(6); q7 = angles(7);

TNum = double(subs(T{7}));
FKNum = TNum(1:3,4);
end

function JNum = calcJ(J, angles)
%{ 
    Takes - J - symbolic robot Jacobian
            angles - 7x1 vecotr of q values
    Returns - JNum - numeric robot Jacobian
%}

%Assign Symbolic Variables & Calculate JNum
q1 = angles(1); q2 = angles(2); q3 = angles(3); q4 = angles(4); q5 = angles(5); q6 = angles(6); q7 = angles(7);

JNum = double(subs(J));
end

function q_found = num_IK(J_sym, T, qi, pdes)
%{
    Takes - J_sym - Symbolic robot Jacobian
            T - cell array of Transformation matrices wrt 0
            qi - 7x1 vector - initial guess of robot q values
            pdes - 3x1 vector of desired end effector position
    Returns - q_found - 7x1 vector of robot q values
%}
threshold = 0.01;

pe = calcFK(T, qi);

error = pdes-pe

while (abs(double(error(1))) > threshold) || (abs(double(error(2))) > threshold) || (abs(double(error(3))) >  threshold)
    Ji = calcJ(J_sym, qi);
    Jv = Ji(1:3,:);
    delta_q = pinv(Jv)*error;
    qi = qi + delta_q;
    
    pe = calcFK(T, qi);

    error = pdes-pe
    disp(qi)
end

q_found = qi;

end

%% Symbolic Model Functions

function T = getSymbolicTMatrices()
%{
    Returns - T - cell array of all transformation matrices wrt {0}
%}
n = 7;
syms q1 q2 q3 q4 q5 q6 q7
q = [q1; q2; q3; q4; q5; q6; q7];

% Define Robot geometry constants
d1 = 0.33;
d3 = 0.316;
d5 = 0.384;
df = 0.107;
a4 = 0.0825;
a5 = -0.0825;
a7 = 0.088;

% Find Transformation Matrices
% Individual Transformation Matrices
T01 = DHtoT(0,0,d1, q(1));
T12 = DHtoT(0, -sym(pi)/2, 0, q(2));
T23 = DHtoT(0, sym(pi)/2, 0, q(3)) * DHtoT(0,0,d3,0);
T34 = DHtoT(a4, sym(pi)/2, 0, q(4));
T45 = DHtoT(0, -sym(pi)/2, 0, q(5)) * DHtoT(a5, 0, d5, 0);
T56 = DHtoT(0, sym(pi)/2, 0, q(6));
T67 = DHtoT(a7, sym(pi)/2, 0, q(7));
T78 = DHtoT(0,0,df,0);

% Note: T78 is a rigid transform with no joint variable, consider T68 as the 7th joint frame
T68 = T67*T78;

%T of each joint in {0}
T02 = T01*T12;
T03 = T01*T12*T23;
T04 = T01*T12*T23*T34;
T05 = T01*T12*T23*T34*T45;
T06 = T01*T12*T23*T34*T45*T56;
T08 = T01*T12*T23*T34*T45*T56*T68;

% Store Transformation matrices
T = cell(n,1); T{1} = T01; T{2} = T02; T{3} = T03; T{4} = T04;
T{5} = T05; T{6} = T06; T{7} = T08;
end

function tau =  calcSymbolicDynamics(T)
%{
    Takes - T - cell array of transformation matrices wrt {0}
    Returns - tau - the symbolic dynamical model of the F.E.P.
%}

% Define Dynamic Constants
n = 7;
syms q1 q2 q3 q4 q5 q6 q7 q1d q2d q3d q4d q5d q6d q7d q1dd q2dd q3dd q4dd q5dd q6dd q7dd
q = [q1; q2; q3; q4; q5; q6; q7];
qdot = [q1d; q2d; q3d; q4d; q5d; q6d; q7d];
qdotdot = [q1dd; q2dd; q3dd; q4dd; q5dd; q6dd; q7dd];

m = [2.34471; 2.36414; 2.38050; 2.42754; 3.49611; 1.46736; 0.45606];
I = [0;0;0;0;0;0;0];                            % I = 0 for point-mass model
gvec = [0;0;9.8];                               % z-direction

% Find the linear velocity Jacobians at each COM
disp("Calculating Jacobians");

Jv1 = [diff(T{1}(1:3,4), q1) diff(T{1}(1:3,4), q2) diff(T{1}(1:3,4), q3) diff(T{1}(1:3,4), q4) diff(T{1}(1:3,4), q5) diff(T{1}(1:3,4), q6) diff(T{1}(1:3,4), q7)];
Jv2 = [diff(T{2}(1:3,4), q1) diff(T{2}(1:3,4), q2) diff(T{2}(1:3,4), q3) diff(T{2}(1:3,4), q4) diff(T{2}(1:3,4), q5) diff(T{2}(1:3,4), q6) diff(T{2}(1:3,4), q7)];
Jv3 = [diff(T{3}(1:3,4), q1) diff(T{3}(1:3,4), q2) diff(T{3}(1:3,4), q3) diff(T{3}(1:3,4), q4) diff(T{3}(1:3,4), q5) diff(T{3}(1:3,4), q6) diff(T{3}(1:3,4), q7)];
Jv4 = [diff(T{4}(1:3,4), q1) diff(T{4}(1:3,4), q2) diff(T{4}(1:3,4), q3) diff(T{4}(1:3,4), q4) diff(T{4}(1:3,4), q5) diff(T{4}(1:3,4), q6) diff(T{4}(1:3,4), q7)];
Jv5 = [diff(T{5}(1:3,4), q1) diff(T{5}(1:3,4), q2) diff(T{5}(1:3,4), q3) diff(T{5}(1:3,4), q4) diff(T{5}(1:3,4), q5) diff(T{5}(1:3,4), q6) diff(T{5}(1:3,4), q7)];
Jv6 = [diff(T{6}(1:3,4), q1) diff(T{6}(1:3,4), q2) diff(T{6}(1:3,4), q3) diff(T{6}(1:3,4), q4) diff(T{6}(1:3,4), q5) diff(T{6}(1:3,4), q6) diff(T{6}(1:3,4), q7)];
Jv7 = [diff(T{7}(1:3,4), q1) diff(T{7}(1:3,4), q2) diff(T{7}(1:3,4), q3) diff(T{7}(1:3,4), q4) diff(T{7}(1:3,4), q5) diff(T{7}(1:3,4), q6) diff(T{7}(1:3,4), q7)];


% Find the angular velocity Jacobians at each COM
Jw1 = [T{1}(1:3,3) [0;0;0] [0;0;0] [0;0;0] [0;0;0] [0;0;0] [0;0;0]];
Jw2 = [T{1}(1:3,3) T{2}(1:3,3) [0;0;0] [0;0;0] [0;0;0] [0;0;0] [0;0;0]];
Jw3 = [T{1}(1:3,3) T{2}(1:3,3) T{3}(1:3,3) [0;0;0] [0;0;0] [0;0;0] [0;0;0]];
Jw4 = [T{1}(1:3,3) T{2}(1:3,3) T{3}(1:3,3) T{4}(1:3,3) [0;0;0] [0;0;0] [0;0;0]];
Jw5 = [T{1}(1:3,3) T{2}(1:3,3) T{3}(1:3,3) T{4}(1:3,3) T{5}(1:3,3) [0;0;0] [0;0;0]];
Jw6 = [T{1}(1:3,3) T{2}(1:3,3) T{3}(1:3,3) T{4}(1:3,3) T{5}(1:3,3) T{6}(1:3,3) [0;0;0]];
Jw7 = [T{1}(1:3,3) T{2}(1:3,3) T{3}(1:3,3) T{4}(1:3,3) T{5}(1:3,3) T{6}(1:3,3) T{7}(1:3,3)];

% Store in cell array
Jvi = cell(n,1); Jvi{1} = Jv1; Jvi{2} = Jv2; Jvi{3} = Jv3; Jvi{4} = Jv4; Jvi{5} = Jv5; Jvi{6} = Jv6; Jvi{7} = Jv7;
Jwi = cell(n,1); Jwi{1} = Jw1; Jwi{2} = Jw2; Jwi{3} = Jw3; Jwi{4} = Jw4; Jwi{5} = Jw5; Jwi{6} = Jw6; Jwi{7} = Jw7;

% Find D(q)
disp("Calculating D(q)");
for i = 1:n
    Kvterm = m(i)*Jvi{i}.'*Jvi{i};
    Kwterm = Jwi{i}.'*T{i}(1:3,1:3)*I(i)*T{i}(1:3,1:3).'*Jwi{i};
    
    if i == 1
        Dq = Kvterm + Kwterm;
    else
        Dq = Dq + Kvterm + Kwterm;
    end
end

% Find Christoffel symbols
disp("Calculating Christoffel symbols");
for i = 1:n
    for j = 1:n
        for k = 1:n
            termone = diff(Dq(k,j), q(i));
            termtwo = diff(Dq(k,i), q(j));
            termthree = diff(Dq(i,j), q(k));
            
            C{i}{j}{k} = 0.5*(termone + termtwo - termthree);
        end
    end
end

% Find C Matrix
disp("Calculating C Matrix");
for k = 1:n
    for j = 1:n
        for i = 1:n
            if i == 1
                Co(k,j) = C{i}{j}{k}*qdot(i);
            else
                Co(k,j) = Co(k,j) + C{i}{j}{k}*qdot(i);
            end
        end
    end
end

% Potential Energy and Gravity term g(q)
disp("Calculating G term");
P1 = m(1) * gvec.' * T{1}(1:3,4);
P2 = m(2) * gvec.' * T{2}(1:3,4);
P3 = m(3) * gvec.' * T{3}(1:3,4);
P4 = m(4) * gvec.' * T{4}(1:3,4);
P5 = m(5) * gvec.' * T{5}(1:3,4);
P6 = m(6) * gvec.' * T{6}(1:3,4);
P7 = m(7) * gvec.' * T{7}(1:3,4);

P = P1 + P2 + P3 + P4 + P5 + P6 + P7;

Gq = [diff(P, q(1)); diff(P, q(2)); diff(P, q(3)); diff(P, q(4)); diff(P, q(5)); diff(P, q(6)); diff(P, q(7))];

% Form the Dynamical Model

tau = Dq*qdotdot + Co*qdot + Gq;
end

function T = DHtoT(a, alpha, d, theta)
%{
    Takes - DH Parameters (a, alpha, d, theta) (Angles in radians)
    Returns - Transformation matrix Ti,i-1 (Frame i in Frame i-1)
%}

T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
    0 sin(alpha) cos(alpha) d;
    0 0 0 1];
end

function J_sym = symbolicJacobian(T)
%{
    Takes T - cell array of transformation matrices in Frame {0}
    Returns J_sym - symbolic robot Jacobian
%}

% Note: All joints are revolute
pe = T{7}(1:3, 4)

zi = [T{1}(1:3,3) T{2}(1:3,3) T{3}(1:3,3) T{4}(1:3,3) T{5}(1:3,3) T{6}(1:3,3) T{7}(1:3,3)] %zi-1 vectors

pi = [T{1}(1:3,4) T{2}(1:3,4) T{3}(1:3,4) T{4}(1:3,4) T{5}(1:3,4) T{6}(1:3,4) T{7}(1:3,3)] %pi-1 vectors

for i = 1:7
    J_sym(1:3,i) = cross(zi(:,i), pe-pi(:,i));
    J_sym(4:6,i) = zi(i);
end

end

function configureMatlab()
%{
    Function to perform all symbolic calculations and save in .mat file for
    future use
%}
T = getSymbolicTMatrices();
tau = calcSymbolicDynamics(T);
J = symbolicJacobian(T);
save('Panda Config');
end
