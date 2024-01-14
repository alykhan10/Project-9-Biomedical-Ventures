Section A: Forward Kinematics Code

% Forward Kinematics Computation of the Robot.

% Initializing the Variables.
syms dis1 dis3 dis4 dis6 dis7 dis9 dis10
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta8 theta9

% Computing the Transformation Matrix of Spherical Joint 1.
T1 = 
[cos(theta1) 0 -sin(theta1) 0; sin(theta1) 0 cos(theta1) 0; 0 -1 0 dis1; 0 0 0 1];
T2 = 
[cos(theta2) 0 sin(theta2) 0; sin(theta2) 0 -cos(theta2) 0; 0 1 0 0; 0 0 0 1];
T3 = 
[cos(theta3) 0 sin(theta3) 0; sin(theta3) 0 -cos(theta3) 0; 0 1 0 dis3; 0 0 0 1];

TSPH1 = T1*T2*T3;

% Computing the Transformation Matrix of Spherical Joint 2.
T4 = 
[cos(theta4) 0 -sin(theta4) 0; sin(theta4) 0 cos(theta4) 0; 0 -1 0 dis4; 0 0 0 1];
T5 = 
[cos(theta5) 0 sin(theta5) 0; sin(theta5) 0 -cos(theta5) 0; 0 1 0 0; 0 0 0 1];
T6 = 
[cos(theta6) 0 sin(theta6) 0; sin(theta6) 0 -cos(theta6) 0; 0 1 0 dis6; 0 0 0 1];

TSPH2 = T4*T5*T6;

% Computing the Transformation Matrix of Spherical Joint 3.
T7 = 
[cos(theta7) 0 -sin(theta7) 0; sin(theta7) 0 cos(theta7) 0; 0 -1 0 dis7; 0 0 0 1];
T8 = 
[cos(theta8) 0 sin(theta8) 0; sin(theta8) 0 -cos(theta8) 0; 0 1 0 0; 0 0 0 1];
T9 = 
[cos(theta9) -sin(theta9) 0 0; sin(theta9) cos(theta9) 0 0; 0 0 1 dis9; 0 0 0 1];

TSPH3 = T7*T8*T9;

% Computing the Transformation Matrix of the Prismatic Joint.
T10 = [1 0 0 0; 0 1 0 0; 0 0 1 dis10; 0 0 0 1];

TPRSM = T10;

% Computing the Resultant Transformation Matrix of the Robot.
TROBOT = TSPH1*TSPH2*TSPH3*TPRSM;

Section B: Jacobian Matrix Code

% Jacobian Computation of the Robot.

% Obtaining the Z-Axes Vector from the Resultant Transformation Matrix of the Robot.
ZT = TROBOT(:,3);

% Obtaining the Joint Variables Vector of the Robot.
QVAR = [theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta8 theta9 dis10];

% Computing the Linear Velocity Jacobian of the Robot.
JV = jacobian(ZT,QVAR);

% Computing the Base Transformation Matrices of Spherical Joint 1.
BASET0 = eye(4);
BASET1 = T1;
BASET2 = BASET1*T2;
BASET3 = BASET2*T3;

% Computing the Base Transformation Matrices of Spherical Joint 2.
BASET4 = BASET3*T4;
BASET5 = BASET4*T5;
BASET6 = BASET5*T6;

% Computing the Base Transformation Matrices of Spherical Joint 3.
BASET7 = BASET6*T7;
BASET8 = BASET7*T8;
BASET9 = BASET8*T9;

% Obtaining the Z-Axes Vectors from the Base Transformation Matrices of Spherical Joint 1.
Z0 = BASET0(:,3);
Z1 = BASET1(:,3);
Z2 = BASET2(:,3);
Z3 = BASET3(:,3);

% Obtaining the Z-Axes Vectors from the Base Transformation Matrices of Spherical Joint 2.
Z4 = BASET4(:,3);
Z5 = BASET5(:,3);
Z6 = BASET6(:,3);

% Obtaining the Z-Axes Vectors from the Base Transformation Matrices of Spherical Joint 3.
Z7 = BASET7(:,3);
Z8 = BASET8(:,3);
Z9 = BASET9(:,3);

% Compiling the Z-Axes Vectors into a Matrix.
ZMATRIX = [Z0 Z1 Z2 Z3 Z4 Z5 Z6 Z7 Z8 Z9];


% Obtaining the Coefficients for Spherical Joint 1.
P1 = [1];
P2 = [1];
P3 = [1];

% Obtaining the Coefficients for Spherical Joint 2.
P4 = [1];
P5 = [1];
P6 = [1];

% Obtaining the Coefficients for Spherical Joint 3.
P7 = [1];
P8 = [1];
P9 = [1];

% Obtaining the Coefficients for the Prismatic Joint.
P10 = [0];

% Compiling the Coefficients into a Vector.
PVECTOR = [P1 P2 P3 P4 P5 P6 P7 P8 P9 P10];

% Computing the Angular Velocity Jacobian of the Robot.
JW = PVECTOR.*ZMATRIX;

% Computing the Full Velocity Jacobian of the Robot.
JFULL = [JV;JW]; 

Section C: Manipulable Workspace Code

% Load Robot, Kinova Gen 3 Robot Used as it Most Closely Resembles our Design.

robot = loadrobot("kinovaGen3", DataFormat="column");

% Plot Robot and Workspace in 3D.

show(robot)
[x, y, z] = sphere(50);
sphereRadius = 1.2;
hold on
p = surf(x * sphereRadius, y * sphereRadius, z * sphereRadius);
set(p, 'FaceColor', 'blue', 'FaceAlpha', 0.3, 'EdgeColor', 'none');

%Trajectory Generation Code from MATLAB Robotics Systems Toolbox.

%{
time = 0:5;
tvec = time(1):1/10:time(end);
numSamples = length(tvec);
rng default
frankaWaypoints = [robot.homeConfiguration robot.homeConfiguration robot.homeConfiguration];
frankaTimepoints = linspace(tvec(1), tvec(end), 3);
[q, qd] = trapveltraj(frankaWaypoints, numSamples);
figure
set(gcf,"Visible","on");
rc = rateControl(10);
for i = 1:numSamples
   hold on
   show(robot, q(:, i), FastUpdate=true, PreservePlot=false);
   waitfor(rc);
end
}%
