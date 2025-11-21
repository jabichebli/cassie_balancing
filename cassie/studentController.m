function tau = studentController(t, s, model, params)

q = s(1 : model.n);
dq = s(model.n+1 : 2*model.n);

% --- Wrench and Force Calculation (Gravity Comp Only) ---
[p_CoM, ~] = computeComPosVel(q, dq, model);
W_des = [0; 0; params.mass * 9.81; 0; 0; 0];

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
F_total = pinv(G_c) * W_des;


% --- Torque Calculation (Corrected based on all findings) ---
[J1f, J1b, J2f, J2b] = computeFootJacobians(s, model);
J_feet_linear = {J1f(1:3,:), J1b(1:3,:), J2f(1:3,:), J2b(1:3,:)};

% 1. Calculate the total 16x1 torque contribution for independent coordinates
tau_dy = zeros(length(model.independent_idx), 1);
for i = 1:num_contacts
    f_i = F_total((i-1)*3+1 : i*3);
    J_i_linear = J_feet_linear{i};
    tau_dy = tau_dy + J_i_linear' * f_i;
end

% 2. Place the independent torques into the full torque vector, applying the
%    negative sign convention from the user's notes.
tau_full = zeros(model.n, 1);
tau_full(model.independent_idx) = -tau_dy;

% 3. Extract torques for actuated joints
tau = tau_full(model.actuated_idx);


% --- DEBUG PRINTING AT T=0 ---
if t == 0
    disp('--- DEBUG: Full Torque Calculation Pipeline (t=0) ---');

    fprintf('Robot Mass: %.2f kg\n', params.mass);
    disp('1. Desired Wrench (W_des):');
    disp(W_des');

    disp('2. Calculated Contact Forces (F_total):');
    F_total_reshaped = reshape(F_total, 3, 4)' ;
    disp('      fx        fy        fz');
    disp(F_total_reshaped);
    sum_F = sum(F_total_reshaped, 1);
    fprintf('Sum of Forces: fx=%.2f, fy=%.2f, fz=%.2f\n', sum_F(1), sum_F(2), sum_F(3));

    disp('3. Independent Torques (tau_dy, before sign flip):');
    disp(tau_dy');

    disp('4. Final Actuated Torques (tau):');
    disp(tau');

    disp('----------------------------------------------------');
end

end
