%% Odometric calibration on REAL data (MADS bag) - Antonelli & Chiaverini
%   - encoder tics come from the "driver" topic of the bag
%   - start/end poses come from the HTC tracker 
%   - the "trajectories" are short pieces (segments) of the recording
%
% Exported the bag data to csv files
clear all; close all; clc;
 
%% STEP 1 - Load the data
name = 'test_3_1';
D = readmatrix([name '_driver.csv'], 'NumHeaderLines', 1);
H = readmatrix([name '_htc.csv'],    'NumHeaderLines', 1);
 
td = D(:,1);  nL = D(:,2);  nR = D(:,3);   % time [s], CUMULATIVE tics
th = H(:,1);  xh = H(:,2);  yh = H(:,3);  yaw = H(:,4);  % HTC pose, yaw unwrapped
 
Enc_res = 4096;          % [tic/rev]
 
% Uses the first part of the recording to calibrate, the rest to validate
t_split = 38;            % [s]
 
%% STEP 2 - Cut the calibration part into segments ("trajectories")
seg_len  = 1.0;                          % segment length [s] 
step     = round(seg_len / median(diff(th)));   % HTC samples per segment
min_rot  = 0.3;                          % skip segments where the wheels turn < 0.3 rev
max_rate = 90*pi/180;                    % skip HTC glitches: |dyaw| > 90 deg/s
 
NTic_R = {};  NTic_L = {};  Pose0 = [];  PoseN = [];
for a = 1:step:numel(th)-step
    b = a + step;
    if th(b) > t_split, break; end
 
    % encoder counts at the HTC start/end times + all driver samples between
    idx = find(td > th(a) & td < th(b));
    tt  = [th(a); td(idx); th(b)];
    cL  = interp1(td, nL, tt);
    cR  = interp1(td, nR, tt);
    dL  = diff(cL)';                     % tics per cycle (row vector)
    dR  = diff(cR)';
 
    % skip segments with (almost) no motion or with an HTC jump
    if (abs(sum(dL)) + abs(sum(dR))) / Enc_res < min_rot, continue; end
    if abs(yaw(b) - yaw(a)) > max_rate * seg_len,        continue; end
 
    NTic_L{end+1,1} = dL;                %#ok<SAGROW>
    NTic_R{end+1,1} = dR;                %#ok<SAGROW>
    Pose0(end+1,:)  = [xh(a) yh(a) yaw(a)];  %#ok<SAGROW>
    PoseN(end+1,:)  = [xh(b) yh(b) yaw(b)];  %#ok<SAGROW>
end
P = numel(NTic_R);
fprintf('%d calibration segments of %.1f s\n', P, seg_len);
 
%% STEP 3 - LS on the orientation: alpha_R = rR/b, alpha_L = rL/b
Phi_th = zeros(P,2);
y_th   = zeros(P,1);
for p = 1:P
    phiR = 2*pi*NTic_R{p}/Enc_res;
    phiL = 2*pi*NTic_L{p}/Enc_res;
    Phi_th(p,:) = [sum(phiR), -sum(phiL)];
    y_th(p)     = PoseN(p,3) - Pose0(p,3);
end
alpha   = Phi_th \ y_th;
alpha_R = alpha(1);
alpha_L = alpha(2);
 
%% STEP 4 - LS on the position: rL
% Real robot: use the half-way heading (eq. 13 in the paper)
Phi_xy = zeros(2*P,1);
y_xy   = zeros(2*P,1);
for p = 1:P
    phiR = 2*pi*NTic_R{p}/Enc_res;
    phiL = 2*pi*NTic_L{p}/Enc_res;
    dth  = alpha_R*phiR - alpha_L*phiL;
    th_k = Pose0(p,3) + [0, cumsum(dth(1:end-1))] + dth/2;   % half-way heading
    s    = (alpha_R/alpha_L*phiR + phiL)/2;
    Phi_xy(2*p-1) = sum(s .* cos(th_k));
    Phi_xy(2*p)   = sum(s .* sin(th_k));
    y_xy(2*p-1)   = PoseN(p,1) - Pose0(p,1);
    y_xy(2*p)     = PoseN(p,2) - Pose0(p,2);
end
rL_hat = Phi_xy \ y_xy;
 
%% STEP 5 - Physical parameters
rR_hat = alpha_R/alpha_L * rL_hat;
b_hat  = rL_hat / alpha_L;
 
fprintf('wheel radius right rR = %.4f m\n', rR_hat);
fprintf('wheel radius left  rL = %.4f m\n', rL_hat);
fprintf('mean wheel radius     = %.4f m\n', (rR_hat + rL_hat)/2);
fprintf('baseline           b  = %.4f m\n', b_hat);
fprintf('cond(Phi_theta) = %.1f\n', cond(Phi_th));
 
%% STEP 6 - Validate on the REST of the recording (not used above)
% Dead reckoning with the calibrated parameters, started at the HTC pose,
% compared with the HTC path.
iv0 = find(th >= t_split, 1);                     % first HTC sample of validation
idx = find(td > th(iv0));
tt  = [th(iv0); td(idx)];
dL  = diff(interp1(td, nL, tt));
dR  = diff(interp1(td, nR, tt));
 
robot = DiffDriveSimulator([b_hat rR_hat rL_hat], [0 0 0], Enc_res, ...
                           [xh(iv0) yh(iv0) yaw(iv0)]);
% NOTE: SimulateEnc divides by dt and multiplies by dt again, so dt cancels.
% A constant dt avoids a division by zero if two timestamps are equal.
for i = 1:numel(dL)
    robot.SimulateEnc(0.01, dR(i), dL(i));
end
odo = robot.GrabPath;                              % one row per element of tt
delete(robot)
 
% Error against HTC at the HTC time stamps
iv  = iv0:numel(th);
xo  = interp1(tt, odo(:,1), th(iv));
yo  = interp1(tt, odo(:,2), th(iv));
err = hypot(xo - xh(iv), yo - yh(iv));
dist = sum(hypot(diff(xh(iv)), diff(yh(iv))));
fprintf('Validation: %.1f m driven, end error %.3f m, max error %.3f m\n', ...
        dist, err(end), max(err));
 
figure(30); hold on; grid on; axis equal
h1 = plot(xh, yh, 'Color', [0.7 0.7 0.7]);
h2 = plot(xh(iv), yh(iv), 'k', 'LineWidth', 2);
h3 = plot(odo(:,1), odo(:,2), 'b--', 'LineWidth', 1.5);
legend([h1 h2 h3], {'HTC, whole recording', 'HTC, validation part', ...
                    'odometry with calibrated parameters'})
xlabel('x [m]'); ylabel('y [m]')
title(sprintf('rR = %.4f, rL = %.4f, b = %.4f m', rR_hat, rL_hat, b_hat))