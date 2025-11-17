function tau = studentController(t, s, model, params)

q = s(1 : model.n);
dq = s(model.n+1 : 2*model.n);

% Get current CoM position and velocity
[p_CoM, v_CoM] = computeComPosVel(q, dq, model);

% Get current pelvis orientation and angular velocity
yaw   = q(4);
pitch = q(5);
roll  = q(6);
R_pelvis = rot_z(yaw) * rot_y(pitch) * rot_x(roll);
w_pelvis = dq(4:6);

% Desired states
p_CoM_des    = params.p_CoM_des;
R_pelvis_des = params.R_pelvis_des;

% Calculate errors
error_p_CoM = p_CoM_des - p_CoM;
error_v_CoM = -v_CoM;
error_R_pelvis = 0.5 * vee_map(R_pelvis_des' * R_pelvis - R_pelvis' * R_pelvis_des);
error_w_pelvis = -w_pelvis;

% Calculate desired wrench
F_des = params.kp_CoM .* error_p_CoM + params.kd_CoM .* error_v_CoM;
T_des = params.kp_pelvis .* error_R_pelvis + params.kd_pelvis .* error_w_pelvis;
W_des = [F_des; T_des];

% Gravity
F_gravity_total = [0; 0; params.mass * 9.81];
F_gravity_per_contact = F_gravity_total / 4;
F_gravity_ff = [F_gravity_per_contact; F_gravity_per_contact; F_gravity_per_contact; F_gravity_per_contact];

% Get foot positions
[p1, p2, p3, p4] = computeFootPositions(q, model);
p_feet = [p1, p2, p3, p4];

% Grasp matrix
A = zeros(6, 12);
for i = 1:4
	A(1:3, (i-1)*3+1:i*3) = eye(3);
	A(4:6, (i-1)*3+1:i*3) = hat_map(p_feet(:,i) - p_CoM);
end

% Calculate desired contact forces
F_corrective = pinv(A) * W_des;

F_total = F_corrective + F_gravity_ff;

% Foot Jacobians
[J1, J2, J3, J4] = computeFootJacobians(s, model);

J1 = J1(1:3, :);
J2 = J2(1:3, :);
J3 = J3(1:3, :);
J4 = J4(1:3, :);

% Compute torques
tau_actuated_full = zeros(16, 1);
tau_actuated_full = tau_actuated_full + J1' * F_total(1:3);
tau_actuated_full = tau_actuated_full + J2' * F_total(4:6);
tau_actuated_full = tau_actuated_full + J3' * F_total(7:9);
tau_actuated_full = tau_actuated_full + J4' * F_total(10:12);

tau_full = zeros(model.n, 1);
tau_full(5:20) = tau_actuated_full;

tau = tau_full(model.actuated_idx);

end
