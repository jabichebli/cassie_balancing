function tau = studentController(t, s, model, ~)
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

kp = 1800 ; 
kd = 300 ;
x0 = getInitialState(model);
q0 = x0(1:model.n) ;
tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx) ;



end