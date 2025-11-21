function tau = studentController(t, s, model, params)

q = s(1 : model.n);
dq = s(model.n+1 : 2*model.n);

% kp = 1800 ; 
% kd = 300 ;
% x0 = getInitialState(model);
% q0 = x0(1:model.n) ;
% tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx) ;


% Get current CoM position and velocity
[p_CoM, v_CoM] = computeComPosVel(q, dq, model);

% Get current pelvis orientation and angular velocity
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

% Position and velocity errors
error_p_CoM = p_CoM - p_CoM_des;
error_v_CoM = v_CoM - v_CoM_des;

% Orientation and angular velocity errors
error_R_pelvis = 0.5 * vee_map(R_pelvis_des' * R_pelvis - R_pelvis' * R_pelvis_des);
error_w_pelvis = w_pelvis - w_pelvis_des;

% Calculate desired wrench
f_d = -params.kp_CoM .* error_p_CoM - params.kd_CoM .* error_v_CoM ...
    + [0; 0; params.mass * 9.81] + params.mass * a_CoM_des;

tau_d = -params.kp_pelvis .* error_R_pelvis - params.kd_pelvis .* error_w_pelvis ...
    + params.I_pelvis * dw_pelvis_des;

W_des = [f_d; tau_d];


% Get foot positions and assemble grasp matrix
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

% Set up QP problem
% Equality constraint: G_c * F_total = W_des
A_eq = G_c;
b_eq = W_des;

% Inequality constraints for friction cones (A_ineq * F <= b_ineq)
mu = params.mu; % Friction coefficient
A_foot = [ 0,  0, -1;  % -fz <= 0 (unilateral force)
           1,  0, -mu; % fx - mu*fz <= 0
          -1,  0, -mu; % -fx - mu*fz <= 0
           0,  1, -mu; % fy - mu*fz <= 0
           0, -1, -mu];% -fy - mu*fz <= 0
A_ineq = kron(eye(num_contacts), A_foot);
b_ineq = zeros(size(A_ineq, 1), 1);

% Objective function: Minimize the squared sum of forces
H = 2 * eye(contact_forces_dim);
f_obj = zeros(contact_forces_dim, 1);
options = optimoptions('quadprog', 'Display', 'none');

% Solve QP
[F_total, ~, exitflag] = quadprog(H, f_obj, [], [], A_eq, b_eq, [], [], [], options);

if exitflag ~= 1
    warning('QP for force distribution: exitflag %d. Infeasible. Using pseudo-inverse fallback.', exitflag);
    F_total = pinv(G_c) * W_des;
end

% computeFootJacobians returns Jacobians in the local body frame of the foot.
[J1f_b, J1b_b, J2f_b, J2b_b] = computeFootJacobians(s, model);
% TODO: is this correct to only use first 3 rows?
J_feet_body_linear = {J1f_b(1:3,:), J1b_b(1:3,:), J2f_b(1:3,:), J2b_b(1:3,:)};

% Get world-frame transforms to extract rotation matrices.
X_w_f1 = bodypos(model, model.idx.foot1, q);
X_w_f2 = bodypos(model, model.idx.foot2, q);
R_w_f1 = X_w_f1(1:3, 1:3);
R_w_f2 = X_w_f2(1:3, 1:3);

% Calculate the total torque contribution for independent coordinates
tau_dy = zeros(length(model.independent_idx), 1);
for i = 1:num_contacts
    % World-frame force for this contact
    f_i_world = F_total((i-1)*3+1 : i*3);

    % Body-frame Jacobian for this contact
	J_i_body_linear = J_feet_body_linear{i};

    % Select the correct rotation matrix for this foot
    if i <= 2
        R_w_b = R_w_f1;
    else
        R_w_b = R_w_f2;
    end

    % Transform world-frame force into the local body-frame of the foot
    f_i_body = R_w_b' * f_i_world;

    % Calculate torque
	tau_dy = tau_dy + J_i_body_linear' * f_i_body;
end

tau = tau_dy; % negative?

end