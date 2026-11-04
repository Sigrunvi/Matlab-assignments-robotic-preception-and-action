%% Odometric calibration (Antonelli & Chiaverini 2005) with DiffDriveSimulator
% Estimates baseline b and wheel radii rR, rL from:
%   - encoder tics for every cycle (what the robot "knows")
%   - start and end pose of each trajectory (what the "camera" measures)
% The simulator is fed the TRUE parameters K_param + DK_param; the
% estimator never sees DK_param.

clear all; close all; clc;

%% STEP 1 - Robot with unknown (true) parameters
b = 0.5;                 % nominal baseline [m]
R = [0.1 0.1];           % nominal radii [rR rL] [m]
K_param  = [b R];
DK_param = [0.02 0.002 -0.001];   % unknown deviations (b, rR, rL)
Enc_res  = 4096;         % [tic/rev]
Tc       = 0.03;         % cycle time [s]

K_true = K_param + DK_param;      % only used to check the result at the end

%% STEP 2 - Calibration trajectories
% Open arcs with constant curvature, turning both left and right and with
% different curvatures. 
% Each row: [tics right per cycle, tics left per cycle, number of cycles]
traj = [100  60  100;    % left turn
         60 100  100;    % right turn
        100  20   80;    % sharp left turn
         20 100   80;    % sharp right turn
        100  80  120;    % gentle left turn
         80 100  120];   % gentle right turn
P = size(traj,1);

% Storage for the measurements of each trajectory
NTic_R = cell(P,1);   % encoder tics right, per cycle
NTic_L = cell(P,1);   % encoder tics left, per cycle
Pose0  = zeros(P,3);  % "camera" pose at start
PoseN  = zeros(P,3);  % "camera" pose at end
path_p = cell(P,1);   % path

for p = 1:P
    robot = DiffDriveSimulator(K_param, DK_param, Enc_res, [0 0 0]);
    robot.setRobotPose([0 0 0]);
    Pose0(p,:) = robot.GrabPose;

    n = traj(p,3);
    NTic_R{p} = traj(p,1) * ones(1,n);
    NTic_L{p} = traj(p,2) * ones(1,n);
    for i = 1:n
        robot.SimulateEnc(Tc, NTic_R{p}(i), NTic_L{p}(i));
    end

    PoseN(p,:) = robot.GrabPose;
    path_p{p}  = robot.GrabPath;  
    delete(robot)
end

%% STEP 3 - LS on the orientation:  alpha_R = rR/b,  alpha_L = rL/b
% theta_N - theta_0 = alpha_R * sum(phi_R) - alpha_L * sum(phi_L)
% phi = wheel angle increment per cycle = 2*pi*NTic/Enc_res  [rad]
Phi_th = zeros(P,2);
y_th   = zeros(P,1);
for p = 1:P
    phiR = 2*pi*NTic_R{p}/Enc_res;
    phiL = 2*pi*NTic_L{p}/Enc_res;
    Phi_th(p,:) = [sum(phiR), -sum(phiL)];
    y_th(p)     = PoseN(p,3) - Pose0(p,3);
end
alpha = Phi_th \ y_th;          % same as pinv(Phi_th)*y_th
alpha_R = alpha(1);
alpha_L = alpha(2);

%% STEP 4 - LS on the position: rL
% x_N - x_0 = rL * sum( (alpha_R/alpha_L*phi_R + phi_L)/2 * cos(theta_i) )
% y_N - y_0 = rL * sum( (alpha_R/alpha_L*phi_R + phi_L)/2 * sin(theta_i) )
% theta_i is rebuilt from the encoders with the alphas from STEP 3.
% NOTE: DiffDriveSimulator uses the OLD heading theta_k to move the
% position, so we do the same here. On a real robot use the half-way
% heading theta_k + dtheta_k/2 (eq. 13 in the paper).
Phi_xy = zeros(2*P,1);
y_xy   = zeros(2*P,1);
for p = 1:P
    phiR = 2*pi*NTic_R{p}/Enc_res;
    phiL = 2*pi*NTic_L{p}/Enc_res;
    dth  = alpha_R*phiR - alpha_L*phiL;            % heading increment
    th   = Pose0(p,3) + [0, cumsum(dth(1:end-1))];  % heading at start of each cycle
    s    = (alpha_R/alpha_L*phiR + phiL)/2;         % arc length / rL
    Phi_xy(2*p-1) = sum(s .* cos(th));
    Phi_xy(2*p)   = sum(s .* sin(th));
    y_xy(2*p-1)   = PoseN(p,1) - Pose0(p,1);
    y_xy(2*p)     = PoseN(p,2) - Pose0(p,2);
end
rL_hat = Phi_xy \ y_xy;

%% STEP 5 - Back to the physical parameters
rR_hat = alpha_R/alpha_L * rL_hat;
b_hat  = rL_hat / alpha_L;

fprintf('              true        estimated    error\n');
fprintf('baseline b  : %.5f m   %.5f m   %+.2e m\n', K_true(1), b_hat,  b_hat  - K_true(1));
fprintf('radius rR   : %.5f m   %.5f m   %+.2e m\n', K_true(2), rR_hat, rR_hat - K_true(2));
fprintf('radius rL   : %.5f m   %.5f m   %+.2e m\n', K_true(3), rL_hat, rL_hat - K_true(3));
fprintf('cond(Phi_theta) = %.1f\n', cond(Phi_th));

%% STEP 6 - Validate on a DIFFERENT trajectory (S-curve)
NR = [100*ones(1,80) 70*ones(1,80)];
NL = [ 70*ones(1,80) 100*ones(1,80)];
true_robot = DiffDriveSimulator(K_param, DK_param, Enc_res, [0 0 0]);
nom_robot  = DiffDriveSimulator(K_param, [0 0 0], Enc_res, [0 0 0]);
cal_robot  = DiffDriveSimulator([b_hat rR_hat rL_hat], [0 0 0], Enc_res, [0 0 0]);
for i = 1:numel(NR)
    true_robot.SimulateEnc(Tc, NR(i), NL(i));
    nom_robot.SimulateEnc(Tc, NR(i), NL(i));
    cal_robot.SimulateEnc(Tc, NR(i), NL(i));
end
pt = true_robot.GrabPath; pn = nom_robot.GrabPath; pc = cal_robot.GrabPath;
fprintf('Validation end-point error: nominal %.3f m, calibrated %.5f m\n', ...
    norm(pn(end,:)-pt(end,:)), norm(pc(end,:)-pt(end,:)));

figure(20); hold on; grid on; axis equal
for p = 1:P, plot(path_p{p}(:,1), path_p{p}(:,2), 'Color', [0.7 0.7 0.7]); end
plot(pt(:,1), pt(:,2), 'k',  'LineWidth', 2)
plot(pn(:,1), pn(:,2), 'r--','LineWidth', 1.5)
plot(pc(:,1), pc(:,2), 'b:', 'LineWidth', 2)
legend([repmat({''},1,P) {'true', 'nominal odometry', 'calibrated odometry'}])
title('Calibration trajectories (grey) and validation S-curve')