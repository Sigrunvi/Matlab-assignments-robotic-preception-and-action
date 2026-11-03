%% Path Following for a Unicycle Robot
% This example demonstrates the disturbing factor effect using a Robot Simulator. 

% Copyright 2014-2016 The MathWorks, Inc. + prof Mariolino De Cecco

% BUG: The matlab "drive" function works only for x AND y > 0 (i.e. in the first quadrant)

% STEP 1 - Define Parameters
clear all;
close all;
clc;

% Baseline
b = 0.5;
% Wheel radii
R = [0.1 0.1];
% Kinematic Parameters
K_param = [b R];
DK_param = 0*[0.2 0.05 0]; % deviation from ideal parameters


% Robot heading
theta0 = 0;
% wheel encoder resolution [tic/rev]
Enc_res = 4096;

% Robot heading error
theta0 = theta0 + 0*pi/180;


% Set the current location and the goal location of the robot as defined by the path
robotCurrentLocation = [0 0];

% Assume an initial robot orientation [rad]
initialOrientation = theta0;

% Define the current pose for the robot [x y theta]
robotInitialPose = [robotCurrentLocation initialOrientation];


%% STEP 2a - Simulate the robot behaviour with Encoder Tics/cycle

% Initialize/Construct the Robot Simulator
% The inputs are: linear and angular velocities like a unicycle
robot = DiffDriveSimulator(K_param, DK_param, Enc_res, [0 0 0]);
robot.setRobotPose(robotInitialPose);

Pose0 = robot.GrabPose; % initial pose

% Define the total simulation time [s]
Tt = 10;

% Define the While Loop update time [s]:
Tc = 0.05;
% Define the current time [s]:
T = 0;

% initialize generic encoder Tics
NTic = 100 * ones(1,100);
NTic = [NTic, - NTic];

% Define a linear path to plot
Vr = K_param(2) * 2*pi * NTic(2) / (Enc_res * Tc);
path = [0.00    0.00;
    Vr * Tt/2    0];

Nplot = 10; % plot number
Xlim = [-1 3];
Ylim = [-2 2];

% Simulates the robot trajectory:
for i = 1:length(NTic)
     
    % MEASUREMENT: acquire the controller outputs, i.e., the inputs to the robot
    
    % Right and Left wheel encoder Tics
    NTic_r = NTic(i);
    NTic_l = 1 * NTic(i);
    
    % SIMULATION: Simulate the robot using the controller outputs.
    robot.SimulateEnc(Tc, NTic_r, NTic_l);
    robot.Show(path, Nplot, Xlim, Ylim);
    
%     % PERCEPTION: Extract current location information ([X,Y]) from the 
%     % current pose of the robot and ADD NOISE simulating the sensors inaccuracy
%     robotCurrentPose = robot.robotCurrentPose + ...
%         0 * [normrnd(0, 0.01, 1, 2) normrnd(0, 2)*pi/180];
    
    % Re-compute the current time
    T = T + Tc;
        
    waitfor(Tc);
 
end

Pose = robot.GrabPose


% Close simulation.
delete(robot)



%% STEP 2b - Simulate the robot behaviour with wheels velocity

% Robot (ideal) linear [left right] wheels velocity
Vr = 1;
Vl = 0.9;

% Define the total simulation time [s]
Tt = 10;
% Define the While Loop update time [s]:
Tc = 0.05;
% Define the current time [s]:
T = 0;

% Define the desired/ideal path
path = [0.00    0.00;
    Vr * Tt    0];

Nplot = 10; % plot number
Xlim = [-1 11];
Ylim = [-6 6];

% Simulates the robot control LOOP:
while( T < Tt )
     
    % ACTION: Compute the controller outputs, i.e., the inputs to the robot
    
    % Wheels Velocity
    
    % Simulate the robot using the controller outputs.
    robot.Simulate(Tc, Vr, Vl);
    robot.Show(path, Nplot, Xlim, Ylim);
    
%     % PERCEPTION: Extract current location information ([X,Y]) from the 
%     % current pose of the robot and ADD NOISE simulating the sensors inaccuracy
%     robotCurrentPose = robot.robotCurrentPose + ...
%         0 * [normrnd(0, 0.01, 1, 2) normrnd(0, 2)*pi/180];
    
    % Re-compute the current time
    T = T + Tc;
    
    waitfor(Tc);
 
end

% Close simulation.
delete(robot)

