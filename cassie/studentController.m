function tau = studentController(t, s, model, params)
params = studentParams(model);
q = s(1 : model.n);
dq = s(model.n+1 : 2*model.n);

% Get current position and velocity
[p_CoM, v_CoM] = computeComPosVel(q, dq, model);
yaw   = q(4);
pitch = q(5);
roll  = q(6);
R_pelvis = rot_z(yaw) * rot_y(pitch) * rot_x(roll);
w_pelvis = dq(4:6);

% Params
p_CoM_des     = params.p_CoM_des;
v_CoM_des     = params.v_CoM_des;
a_CoM_des     = params.a_CoM_des;
R_pelvis_des  = params.R_pelvis_des;
w_pelvis_des  = params.w_pelvis_des;
dw_pelvis_des = params.dw_pelvis_des;

% Calculate errors
error_p_CoM = p_CoM - p_CoM_des;
error_v_CoM = v_CoM - v_CoM_des;
error_R_pelvis = 0.5 * vee_map(R_pelvis_des' * R_pelvis - R_pelvis' * R_pelvis_des);
error_w_pelvis = w_pelvis - w_pelvis_des;

% Calculate desired wrench
f_d = -params.kp_CoM .* error_p_CoM - params.kd_CoM .* error_v_CoM ...
    + [0; 0; params.mass * 9.81] + params.mass * a_CoM_des;
tau_d = -params.kp_pelvis .* error_R_pelvis - params.kd_pelvis .* error_w_pelvis ...
    + params.I_pelvis * dw_pelvis_des;
W_des = [f_d; tau_d];

% Grasp matrix
[p1, p2, p3, p4] = computeFootPositions(q, model);
p_feet = {p1, p2, p3, p4};
num_contacts = 4;
contact_forces_dim = 3 * num_contacts;
G_c = zeros(6, contact_forces_dim);
for i = 1:num_contacts
    p_foot_relative = p_feet{i} - p_CoM;
    G_c(1:3, (i-1)*3+1:i*3) = eye(3);
    G_c(4:6, (i-1)*3+1:i*3) = hat_map(p_foot_relative);
end

% Set up QP
A_eq = G_c;
b_eq = W_des;
mu = params.mu;
A_foot = [ 0,  0, -1;  % -fz <= 0
    1,  0, -mu; % fx - mu*fz <= 0
    -1,  0, -mu; % -fx - mu*fz <= 0
    0,  1, -mu; % fy - mu*fz <= 0
    0, -1, -mu];% -fy - mu*fz <= 0
A_ineq = kron(eye(num_contacts), A_foot);
b_ineq = zeros(size(A_ineq, 1), 1);

% Solve QP
H = 2 * eye(contact_forces_dim);
f_obj = zeros(contact_forces_dim, 1);
options = optimoptions('quadprog', 'Display', 'none');
[F_total, ~, exitflag] = quadprog(H, f_obj, A_ineq, b_ineq, A_eq, b_eq, [], [], [], options);

if exitflag ~= 1
    error('QP for force distribution became infeasible with exitflag %d. Halting.', exitflag);
end

% computeFootJacobians returns Jacobians in the world frame.
% The linear velocity portion is in rows 4-6.
[J1f_w, J1b_w, J2f_w, J2b_w] = computeFootJacobians(s, model);
J_feet_world_linear = {J1f_w(4:6,:), J1b_w(4:6,:), J2f_w(4:6,:), J2b_w(4:6,:)};

% Torque Calculation
tau_dy = zeros(length(model.independent_idx), 1);
for i = 1:num_contacts
    % Both Jacobian and force are in the world frame, so we can multiply directly.
    f_i_world = F_total((i-1)*3+1 : i*3);
    J_i_world_linear = J_feet_world_linear{i};
    tau_dy = tau_dy + J_i_world_linear' * f_i_world;
end

tau_full = zeros(model.n, 1);
tau_full(model.independent_idx) = -tau_dy;
tau = tau_full(model.actuated_idx);

end