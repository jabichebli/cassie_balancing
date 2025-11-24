function params = studentParams(model)

x0 = getInitialState(model);
q0 = x0(1:model.n);
dq0 = x0(model.n+1:2*model.n);
[p_CoM0, ~] = computeComPosVel(q0, dq0, model);

params = struct();
params.mass = model.M;

params.p_CoM_des    = p_CoM0;
params.R_pelvis_des = eye(3);

params.kp_CoM = 2100 * [0.5; 1; 3]; % 2100  [1; 1; 3]
params.kd_CoM = 195 * [1; 1; 2];  % 195  [1; 1; 2]
params.kp_pelvis = 1000 * [1; 1; 0.1]; 
params.kd_pelvis = 0.2 * [1; 1; 1];

% params.kp_CoM = [1000; 1000; 1000];
% params.kd_CoM = [100; 100; 100];
% params.kp_pelvis = [100 100 100]';
% params.kd_pelvis = [10 10 10]';

params.kd_internal = 5.0;


% Desired velocities and accelerations
params.v_CoM_des    = [0; 0; 0];
params.a_CoM_des    = [0; 0; 0];
params.w_pelvis_des = [0; 0; 0];
params.dw_pelvis_des = [0; 0; 0];

% Robot properties
params.I_pelvis = model.I{6}(1:3, 1:3);
params.mu = 0.8;

% -----------------------------------------------------------------------
% STEP PARAMS 
% -----------------------------------------------------------------------
params.foot_dx = 0;
params.foot_dy = 0.2;

params.t1 = 0.4;
params.t2 = 0.5;

params.tolerance = 0.05; % 0.05

end