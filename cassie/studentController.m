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


% --- New Friction Cone Formulation ---
n_friction_edges = 4;
ths = linspace(0, 2*pi, n_friction_edges + 1);
ths = ths(1:n_friction_edges); % Use n unique angles

% Create the matrix of cone edge vectors (V) for one foot
V = zeros(3, n_friction_edges);
for i = 1:n_friction_edges
    V(:, i) = [params.mu * cos(ths(i)); params.mu * sin(ths(i)); 1];
end

% Create the block-diagonal V_block matrix for all 4 feet
V_block = kron(eye(num_contacts), V);

% The new decision variable is alpha_total (n_friction_edges * num_contacts)
dim_alpha = n_friction_edges * num_contacts;


% --- Reformulate QP in terms of alpha ---
% Objective: min ||F_total||^2  => min ||V_block * alpha||^2
H = 2 * (V_block' * V_block);
f_obj = zeros(dim_alpha, 1);

% Equality Constraint: G_c * F_total = W_des  => (G_c * V_block) * alpha = W_des
A_eq = G_c * V_block;

% Inequality Constraint: alpha >= 0 is handled by a lower bound (lb).
A_ineq = [];
b_ineq = [];
lb = zeros(dim_alpha, 1);
ub = [];

options = optimoptions('quadprog', 'Display', 'none', 'MaxIter', 1000);


% --- Iteratively solve QP (for alpha) ---
kp_CoM = params.kp_CoM;
kd_CoM = params.kd_CoM;
kp_pelvis = params.kp_pelvis;
kd_pelvis = params.kd_pelvis;

max_tries = 1;
exitflag = -2;
W_des = [];
alpha_total = []; % The variable we solve for now

for i = 1:max_tries
    % Calculate desired wrench
    f_d = -kp_CoM .* error_p_CoM - kd_CoM .* error_v_CoM ...
        + [0; 0; params.mass * 9.81] + params.mass * a_CoM_des;
    tau_d = -kp_pelvis .* error_R_pelvis - kd_pelvis .* error_w_pelvis ...
        + params.I_pelvis * dw_pelvis_des;
    W_des = [f_d; tau_d];
    b_eq = W_des;

    % Solve QP for alpha_total
    [alpha_total, ~, exitflag] = quadprog(H, f_obj, A_ineq, b_ineq, A_eq, b_eq, lb, ub, [], options);
    
    if exitflag == 1
        break;
    end

    % Reduce gains for next iteration
    kp_CoM = kp_CoM * 0.9;
    kd_CoM = kd_CoM * 0.9;
    kp_pelvis = kp_pelvis* 0.9;
    kd_pelvis = kd_pelvis* 0.9;
end


% --- Handle QP failure and recover F_total from alpha_total ---
if exitflag ~= 1
    % W_des_string = mat2str(W_des);
    % error('QP failed to find a solution. Desired Wrench: %s', W_des_string);
    kp = 1800; 
    kd = 300;
    x0 = getInitialState(model);
    q0 = x0(1:model.n);
    tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx);
    return;
end

% CRITICAL STEP: Recover the contact forces from the solved alpha values
F_total = V_block * alpha_total;

% Jacobians
[J1f_w, J1b_w, J2f_w, J2b_w] = computeFootJacobians(s, model);
J_com_W = computeComJacobian(q, model);
[~, ~, G] = model.gamma_q(model, q, dq);
J_com_W = J_com_W*G;
J_feet_world_linear = {J1f_w(4:6,:) - J_com_W, J1b_w(4:6,:)-J_com_W, J2f_w(4:6,:)-J_com_W, J2b_w(4:6,:)-J_com_W};

% Torque Calculation
tau_dy = zeros(length(model.independent_idx), 1);
for i = 1:num_contacts
    f_i_world = F_total((i-1)*3+1 : i*3);
    J_i_world_linear = J_feet_world_linear{i};
    tau_dy = tau_dy + J_i_world_linear' * f_i_world;
end

tau_full = zeros(model.n, 1);
tau_full(model.independent_idx) = -tau_dy;
tau_model = tau_full(model.actuated_idx);
% Joint Damping
dq_actuated = dq(model.actuated_idx);
tau_damping = -params.kd_internal .* dq_actuated;
tau = tau_model + tau_damping;

if any(abs(tau) > 100) || max([p1(3), p2(3), p3(3), p4(3)]) > 1e-6 && t > 0.01
    kp = 1800; 
    kd = 300;
    x0 = getInitialState(model);
    q0 = x0(1:model.n);
    tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx);
    return;
end

end
