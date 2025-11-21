function tau = studentController(t, s, model, params)

q = s(1 : model.n);
dq = s(model.n+1 : 2*model.n);

% kp = 1800 ; 
% kd = 300 ;
% x0 = getInitialState(model);
% q0 = x0(1:model.n) ;
% tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx) ;
% disp(tau');

% --- Wrench and Force Calculation ---
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
[J1f_b, J1b_b, J2f_b, J2b_b] = computeFootJacobians(s, model);
J_feet_body_linear = {J1f_b(1:3,:), J1b_b(1:3,:), J2f_b(1:3,:), J2b_b(1:3,:)};

% Get world-frame transforms to extract rotation matrices.
% Assuming bodypos returns a 4x4 homogeneous transform.
X_w_f1 = bodypos(model, model.idx.foot1, q);
X_w_f2 = bodypos(model, model.idx.foot2, q);
R_w_f1 = X_w_f1(1:3, 1:3);
R_w_f2 = X_w_f2(1:3, 1:3);


% 1. Calculate the total 16x1 torque contribution for independent coordinates
tau_dy = zeros(length(model.independent_idx), 1);
for i = 1:num_contacts
	% World-frame force for this contact
    f_i_world = F_total((i-1)*3+1 : i*3);

    % Body-frame Jacobian for this contact
	J_i_body_linear = J_feet_body_linear{i};

    % Select the correct rotation matrix to transform the force into the Jacobian's frame
    if i <= 2 % Contacts 1 & 2 are on foot 1
        R_w_b = R_w_f1;
    else % Contacts 3 & 4 are on foot 2
        R_w_b = R_w_f2;
    end

    % Transform world-frame force into the local body-frame of the foot
    % R_w_b' is the inverse rotation (from world to body)
    f_i_body = R_w_b' * f_i_world;

    % Now that J and f are in the same frame, calculate torque contribution
	tau_dy = tau_dy + J_i_body_linear' * f_i_body;
end

% 2. Place the independent torques into the full torque vector, applying the
%    negative sign convention from the user's notes.
tau_full = zeros(model.n, 1);
tau_full(model.independent_idx) = -tau_dy;

% 3. Extract torques for actuated joints
tau = tau_full(model.actuated_idx);

disp(tau');


% % --- DEBUG PRINTING AT T=0 ---
% if t == 0
% 	disp('--- DEBUG: Full Torque Calculation Pipeline (t=0) ---');
% 
% 	fprintf('Robot Mass: %.2f kg\n', params.mass);
% 	disp('1. Desired Wrench (W_des):');
% 	disp(W_des');
% 
% 	disp('2. Calculated Contact Forces (F_total):');
% 	F_total_reshaped = reshape(F_total, 3, 4)' ;
% 	disp('      fx        fy        fz');
% 	disp(F_total_reshaped);
% 	sum_F = sum(F_total_reshaped, 1);
% 	fprintf('Sum of Forces: fx=%.2f, fy=%.2f, fz=%.2f\n', sum_F(1), sum_F(2), sum_F(3));
% 
% 	disp('3. Independent Torques (tau_dy, before sign flip):');
% 	disp(tau_dy');
% 
% 	disp('4. Final Actuated Torques (tau):');
% 	disp(tau');
% 
% 	disp('----------------------------------------------------');
% end

end
