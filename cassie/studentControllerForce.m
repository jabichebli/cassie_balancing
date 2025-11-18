function tau = studentControllerForce(t, s, model, ~)
% Modify this code to calculate the joint torques
% t - time
% s - state of the robot
% model - struct containing robot properties
% params - user defined parameters in studentParams.m
% tau - 10x1 vector of joint torques

% State vector components ID
q = s(1 : model.n);
dq = s(model.n+1 : 2*model.n);

%% [Control #1] zero control
% tau = zeros(10,1);

%% [Control #2] Highest Score so far on leaderboard 

% Leaderboard score  |  Kp  |  Kd
%      218.64        | 1800 |  300
%      218.37        | 3000 |  500
%      163.67        | 1200 |  200 

% kp = 1800 ; 
% kd = 300 ;
% x0 = getInitialState(model);
% q0 = x0(1:model.n) ;
% tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx) ;

%% [Control #3] Force Controller

% Get initial conditons 
x0 = getInitialState(model);
q0 = x0(1:model.n) ;
dq0 = zeros(model.n, 1);

kp_fd = 1800 ; 
kd_fd = 300 ;
kp_td = 0;
kd_td = 0;
g = 9.81; 

[rd_com, vd_com] = computeComPosVel(q0, dq0, model);
[r_com, v_com] = computeComPosVel(q, dq, model);

fd = - kp_fd * (r_com - rd_com) - kd_fd * (v_com - vd_com) + model.M * g * [0; 0; 1];


e_orient = zeros(length(fd),1);
e_ang_vel = zeros(length(fd),1);

taud = - kp_td * e_orient - kd_td * e_ang_vel;

Fd = [fd; taud];

[p1, p2, p3, p4] = computeFootPositions(q, model);


% reaction forces constraints



% friction cone constraints





end