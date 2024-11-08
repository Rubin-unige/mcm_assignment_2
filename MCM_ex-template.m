%% Esercitazione MCM - Cambio d'osservatore e derivte vettori.
addpath('include\');

clc;
clear all;

%% Dati
% Inertial position of the body and the goal
w_r_bw = [2,2,0]';
w_r_gw = [4,5,0]';

% Initial orientation of the <b> frame
w_eta_b = [pi/6,0,0]';%(yaw,pitch,roll)

% Velocities (ang, lin) of the body w.r.t. <w>
b_v_bw = [0.0,0.0,0.0]';%(x,y,z) m/s
b_omega_bw = [0.0,0.0,0.0]';%(x,y,z) rad/sec

% Orientation of the goal w.r.t. the <b> frame 
b_rho_g = [pi/4,0,pi/6];%(x,y,z)

% Velocities (ang, lin) of the goal w.r.t. the <b> frame 
b_v_gb = [0.1,0.0,0.0]';%(x,y,z) m/s
b_omega_gb = [0,0.0,0.0]';%(x,y,z)

%% Exercise 1

% Compute the velocity of <g>
w_omega_bw = a_R_b * b_omega_bw;
w_omega_gw = w_omega_bw + b_omega_gb;
w_v_gw = w_r_gw * w_omega_gw;

%% Exercise 2

% Given a point P rigidly attached to <g>
g_r_Pg = [0.5,0.5,0.0]';

% Comput ethe velocity of P w.r.t. the world
w_v_Pw = ;

%% Exercise 3

% Reach the moving point with <b> and align <b> to <g>
% knowing that:
w_v_gw = [0.5, 0.8, 0.0]';
w_omega_gw = [0.0,0.0,0.0]';
w_r_Pg = g_r_Pg + w_r_gw;
w_eta_P = ...;


% Control

% Simulation variables
% simulation time definition 
dt = 0.01;
t_start = 0.0;
t_end = 20.0;
t = t_start:dt:t_end; 


% init data structures
t_hist = [];
lin_err_hist = [];
ang_err_hist = [];

% Choose the solving method
method1 = false;

for i = t %todo wRg

    % Compute the feed-forward velocities
    w_v_Pw = ...;
    w_omega_Pw = ...;
    
    % Compute cartesian errors
    w_r_Pb = ...; %lin
    w_rho_gb = ...;% ang

    % Apply the ctrl law
    [w_v_bw,w_omega_bw] = cartesianCtrl(w_r_Pb,w_rho_gb,w_v_Pw,w_omega_Pw);
    
    % integrate the positions
   
    
    % integrate the rotations
    if method1 
        % Directly integrate the rotation matrix
        wRb = ...;
        % SVD applied to rotation matrix computation (Procrustes Problem)
        % for orthonormalize the rotation matrix
        wRb = orthonormalize(wRb);
    else
        % Convert the rotation matrix to YPR and integrate the euler
        % angles.
        YPR_rate = ...;
        YPR = ...;
        wRb = YPRToRot(YPR(1),YPR(2),YPR(3));
    end

    disp('Linear Error');
    disp(w_v_bw);
    disp('Angular Error');
    disp(w_omega_bw);

    t_hist = [t_hist; i];
    lin_err_hist = [lin_err_hist; norm(w_r_bg)];
    ang_err_hist = [ang_err_hist; norm(w_rho_gb)];

end

figure
plot(t_hist,lin_err_hist,LineWidth=2)
title('Linear Error')
ylabel('norm(d)')
xlabel('t')

figure
plot(t_hist,ang_err_hist,LineWidth=2)
title('Angular Error')
xlabel('t')
ylabel('norm(rho)')