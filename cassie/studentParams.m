function params = studentParams(model)

x0 = getInitialState(model);
q0 = x0(1:model.n);
dq0 = x0(model.n+1:2*model.n);
[p_CoM0, ~] = computeComPosVel(q0, dq0, model);

params = struct();
params.mass = model.M;

params.p_CoM_des    = p_CoM0;
params.R_pelvis_des = eye(3);

% PD for CoM
params.kp_CoM = 1 * [1; 1; 1];
params.kd_CoM = 1 * [1; 1; 1];

% PD for Orientation
params.kp_pelvis = 1 * [1; 1; 1];
params.kd_pelvis = 1 * [1; 1; 1];

% Desired trajectory (velocities and accelerations)
params.v_CoM_des    = [0; 0; 0];
params.a_CoM_des    = [0; 0; 0];
params.w_pelvis_des = [0; 0; 0];
params.dw_pelvis_des = [0; 0; 0];

% Robot properties
params.I_pelvis = model.I{6}(1:3, 1:3);
params.mu = 0.8;

end
