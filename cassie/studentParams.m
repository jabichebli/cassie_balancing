function params = studentParams(model)

x0 = getInitialState(model);
q0 = x0(1:model.n);
dq0 = x0(model.n+1:2*model.n);
[p_CoM0, ~] = computeComPosVel(q0, dq0, model);

params = struct();
params.mass = 0;
for i = 1:model.NB
	params.mass = params.mass + model.I{i}(6,6);
end

params.p_CoM_des    = p_CoM0;
params.R_pelvis_des = eye(3);

% PD for CoM
params.kp_CoM = 1000 * [1; 1; 1];
params.kd_CoM = 200 * [1; 1; 1];

% PD for Orientation
params.kp_pelvis = 500 * [1; 1; 1];
params.kd_pelvis = 50 * [1; 1; 1];

end
