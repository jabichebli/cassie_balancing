function tau = studentController(t, s, model, params)

q = s(1 : model.n);
dq = s(model.n+1 : 2*model.n);

% Get current CoM position and velocity
[p_CoM, v_CoM] = computeComPosVel(q, dq, model);

% Get current pelvis orientation and angular velocity
yaw   = q(4);
pitch = q(5);
roll  = q(6);
R_pelvis = rot_z(yaw) * rot_y(pitch) * rot_x(roll); % check
w_pelvis = dq(4:6);

% Desired trajectory
p_CoM_des    = params.p_CoM_des;
v_CoM_des    = params.v_CoM_des;
a_CoM_des    = params.a_CoM_des;
R_pelvis_des = params.R_pelvis_des;
w_pelvis_des = params.w_pelvis_des;
dw_pelvis_des = params.dw_pelvis_des;

% Position error
error_p_CoM = p_CoM - p_CoM_des;
error_v_CoM = v_CoM - v_CoM_des;

% Orientation error
error_R_pelvis = 0.5 * vee_map(R_pelvis_des' * R_pelvis - R_pelvis' * R_pelvis_des); % check
error_w_pelvis = w_pelvis - w_pelvis_des;

% Desired force
f_d = -params.kp_CoM .* error_p_CoM - params.kd_CoM .* error_v_CoM ...
    + [0; 0; params.mass * 9.81] + params.mass * a_CoM_des;

% Desired torque
tau_d = -params.kp_pelvis .* error_R_pelvis - params.kd_pelvis .* error_w_pelvis ...
    + params.I_pelvis * dw_pelvis_des;

% Desired wrench
W_des = [f_d; tau_d];

% Get foot positions
[p1, p2, p3, p4] = computeFootPositions(q, model);
p_feet = [p1, p2, p3, p4];

% Grasp matrix
num_contacts = 4;
contact_forces_dim = 3 * num_contacts;
G_c = zeros(6, contact_forces_dim);
for i = 1:num_contacts
    p_foot_relative = p_feet(:,i) - p_CoM;
    G_c(1:3, (i-1)*3+1:i*3) = eye(3);
    G_c(4:6, (i-1)*3+1:i*3) = hat_map(p_foot_relative);
end

% G_c * f_c = W_des
A_eq = G_c;
b_eq = W_des;

% A_ineq * f_c <= b_ineq
mu = params.mu;

% -fz <= 0  (unilateral force)
% fx - mu*fz <= 0 (friction cone)
% -fx - mu*fz <= 0 (friction cone)
% fy - mu*fz <= 0 (friction cone)
% -fy - mu*fz <= 0 (friction cone)
A_foot = [ 0,  0, -1;
    1,  0, -mu;
    -1,  0, -mu;
    0,  1, -mu;
    0, -1, -mu];

% Combine for all feet
A_ineq = kron(eye(num_contacts), A_foot);
b_ineq = zeros(size(A_ineq, 1), 1);

% Solve the optimization problem
H = 2 * eye(contact_forces_dim);
f = zeros(contact_forces_dim, 1);
options = optimoptions('quadprog', 'Display', 'none');

[F_total, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, A_eq, b_eq, [], [], [], options);

if exitflag ~= 1
    warning('QP for force distribution: exitflag %d. Infeasible or had other issues. Using pseudo-inverse fallback.', exitflag);
    F_total = pinv(G_c) * W_des;
end

% Foot Jacobians
[J1, J2, J3, J4] = computeFootJacobians(s, model);
J_feet = {J1(1:3,:), J2(1:3,:), J3(1:3,:), J4(1:3,:)};

tau_full = zeros(model.n, 1);
for i = 1:num_contacts
    force_idx = (i-1)*3+1 : i*3;
    f_i = F_total(force_idx);
    J_i = J_feet{i};
    
    torque_contribution = J_i' * f_i;
    tau_full(model.independent_idx) = tau_full(model.independent_idx) + torque_contribution;
end

tau = tau_full(model.actuated_idx);

end
