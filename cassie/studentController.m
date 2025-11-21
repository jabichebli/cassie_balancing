function out = studentController(t, s, model, params)
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
% kp = 1800 ;
% kd = 300 ;
% x0 = getInitialState(model);
% q0 = x0(1:model.n) ;
% out = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx) ;

%% Force Control

%   TODO:
% check if contact frames are equal to world frames

% desired wrench CHANGE TO COM
f_des_W = -params.Kp_f*(q(1:3) - params.r_com_des_W) - params.Kd_f*(dq(1:3) - params.dr_com_des_W) + params.m*params.g*[0; 0; 1]; % + params.m * ddr_com_d_W;
tau_des_W = -params.Kp_tau*(q(4:6) - params.rot_des) - params.Kd_tau*(dq(4:6) - params.drot_des); % + params.I * ddrot_des

F_des_W = [f_des_W; tau_des_W]; % desired wrench in world frame

% Grasp Matrix
[p_lf_W, p_lb_W, p_rf_W, p_rb_W] = computeFootPositions(q, model); % contact positions in World frame (lf: left front, rb: right back)

[r_com_W, v_com_W] = computeComPosVel(q, dq, model); % position and velocity of COM in world frame

r_lf_W = p_lf_W - r_com_W; % distance between COM and each contact position in world frame
r_lb_W = p_lb_W - r_com_W;
r_rf_W = p_rf_W - r_com_W;
r_rb_W = p_rb_W - r_com_W;


G = [eye(3), eye(3), eye(3), eye(3);
     hat_map(r_lf_W), hat_map(r_lb_W), hat_map(r_rf_W), hat_map(r_rb_W)];

% Optimization for Contact Forces 



fminconoptions = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'Algorithm', 'interior-point', ...
    'MaxFunctionEvaluations', 1e5, ...
    'MaxIterations', 5000);

[f_contacts_W, ~] = fmincon(@cost, params.f_contacts_0_W, [], [], G, F_des_W, [], [], @nonlcon, fminconoptions);

    function J = cost(X)

        J = norm(X)^2;

    end

    function [c, ceq] = nonlcon(X)

        c_uni = [-X(3:3:end)];

        c_friction = zeros(4, 1);
        for i = 1:height(c_friction)
            c_friction(i) = abs(norm(X(i*3-2:i*3-1))/X(3*i)) - 0.8;
        end

        c = [c_uni; c_friction];
        ceq = [];

    end

params.f_contacts_0_W = f_contacts_W;

% Compute Joint Torques

[J_lf, J_lb, J_rf, J_rb] = computeFootJacobians(s,model);


temp = -J_lf'*f_contacts_W(1:3) - J_lb'*f_contacts_W(4:6) - J_rf'*f_contacts_W(7:9) - J_rb'*f_contacts_W(10:12);
out = temp(model.actuated_idx);

end

