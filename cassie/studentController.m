function tau = studentController(t, s, model, params)
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

%% [Control #2] High Gain Joint PD control on all actuated joints
kp = 1000 ;
kd = 250 ;
x0 = getInitialState(model);
q0 = x0(1:model.n) ;
tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx) ;

%% [Control #3] Position control based balancing

%% [Control #4] Contact-Force based balancing

%% [Control #5] Momentum-based balancing

%% [Control #6] Push recovery and stepping based balancing