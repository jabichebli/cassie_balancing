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

% desired wrench
[r_com_W, v_com_W] = computeComPosVel(q, dq, model); % position and velocity of COM in world frame

f_des_W = -params.Kp_f*(r_com_W - params.r_com_des_W) - params.Kd_f*(v_com_W - params.dr_com_des_W) + params.m*params.g*[0; 0; 1]; % + params.m * ddr_com_d_W;
tau_des_W = -params.Kp_tau*(q(4:6) - params.rot_des) - params.Kd_tau*(dq(4:6) - params.drot_des); % + params.I * ddrot_des

F_des_W = [f_des_W; tau_des_W]; % desired wrench in world frame

% Grasp Matrix
[p_lf_W, p_lb_W, p_rf_W, p_rb_W] = computeFootPositions(q, model); % contact positions in World frame (lf: left front, rb: right back)

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

A_uni = zeros(4, 12);
A_uni(1, 3) = -1;
A_uni(2, 6) = -1;
A_uni(3, 9) = -1;
A_uni(4, 12) = -1;

A1 = [eye(3, 3), zeros(3, 3)];
A2 = [zeros(3, 3), eye(3, 3)];
alpha1 = 1;
alpha2 = 10e-3;
alpha3 = 10e-6;

H = 2*alpha1 * (G' * (A1' * A1) * G) ...
  + 2*alpha2 * (G' * (A2' * A2) * G) ...
  + 2*alpha3 * eye(size(G,2));

f = -2*alpha1 * (A1*F_des_W)' * A1 * G ...
  -2*alpha2 * (A2*F_des_W)' * A2 * G;

% H = 2*eye(length(params.f_contacts_0_W));   % Hessian
% f = zeros(size(params.f_contacts_0_W));     % Linear term
A = A_uni;
b = zeros(4,1);
Aeq = G;
beq = F_des_W;

f_contacts_W = quadprog(H, f, A, b, Aeq, beq, [], []);

% [f_contacts_W, ~] = fmincon(@cost, params.f_contacts_0_W, A_uni, zeros(4, 1), G, F_des_W);
% 
%     function J = cost(X)
% 
%         J = norm(X)^2;
% 
%     end
% 
%     function [c, ceq] = nonlcon(X)
% 
%         c_uni = [-X(3:3:end)];
% 
%         c_friction = zeros(4, 1);
%         for i = 1:height(c_friction)
%             c_friction(i) = abs(norm(X(i*3-2:i*3-1))/X(3*i)) - 0.8;
%         end
% 
%         c = [c_uni; c_friction];
%         ceq = [];
% 
%     end
% params.f_contacts_0_W = f_contacts_W;

% Compute Joint Torques

% R = rot_x(q(6))*rot_y(q(5))*rot_z(q(4));
% R = R';
% 
% [J_lf, J_lb, J_rf, J_rb] = computeFootJacobians(s,model);
% J_com = computeComJacobian(q, model);
% 
% J1 = J_lf(4:6, :)-R*J_com(:, model.independent_idx);
% J2 = J_lb(4:6, :)-R*J_com(:, model.independent_idx);
% J3 = J_rf(4:6, :)-R*J_com(:, model.independent_idx);
% J4 = J_rb(4:6, :)-R*J_com(:, model.independent_idx);
% 
% Bi = [zeros(3, 3); eye(3, 3)];
% temp = -J_lf'*Bi*f_contacts_W(1:3) - J_lb'*Bi*f_contacts_W(4:6) - J_rf'*Bi*f_contacts_W(7:9) - J_rb'*Bi*f_contacts_W(10:12);

f_lf_W = f_contacts_W(1:3); f_lb_W = f_contacts_W(4:6); f_rf_W = f_contacts_W(7:9); f_rb_W = f_contacts_W(10:12);

% calculate left/right foot body positions
X_W_foot1 = bodypos(model, model.idx.foot1, q);
X_W_foot2 = bodypos(model, model.idx.foot2, q);
r_foot1 = X_to_r(X_W_foot1);   % world position of foot frame
r_foot2 = X_to_r(X_W_foot2);

F1 = [f_lf_W ; cross(p_lf_W - r_foot1, f_lf_W)]; % convert lf force to wrench at left foot body
F2 = [f_lb_W ; cross(p_lb_W - r_foot1, f_lb_W)]; % convert lb force to wrench at left foot body
F3 = [f_rf_W ; cross(p_rf_W - r_foot2, f_rf_W)]; % convert rf force to wrench at right foot body
F4 = [f_rb_W ; cross(p_rb_W - r_foot2, f_rb_W)]; % convert rb force to wrench at right foot body

f_ext = cell(model.NB,1);
f_ext{model.idx.foot1} = F1 + F2;
f_ext{model.idx.foot2} = F3 + F4;

[H, C] = HandC(model, q, dq, f_ext);

out = C(model.actuated_idx);
display(out);
display(t);

end