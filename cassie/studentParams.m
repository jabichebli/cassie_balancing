function params = studentParams(model)
% define any parameters here 
% params - struct

params = struct();

[x0, model] = getInitialState(model);

q0 = x0(1:20);
dq0 = x0(21:40);

[r_com_0, ~] = computeComPosVel(q0, dq0, model);

params.m  = model.M;
params.g = 9.81;

% params.Kp_f = 1*[900; 900; 3000];
% params.Kd_f = 1*[sqrt(params.m*params.Kp_f(1))*2*0.8; sqrt(params.m*params.Kp_f(1))*2*0.8; sqrt(params.m*params.Kp_f(3))*2*0.2];

params.Kp_f= 1000 * [1; 1; 3]; % 2100  [1; 1; 3]
params.Kd_f = 100 * [1; 1; 2];  % 195  [1; 1; 2]

params.Kp_tau = 500 * [0.3; 0.3; 0.3]; 
params.Kd_tau = 10 * [0.3; 0.3; 0.3];

params.r_com_des_W = r_com_0;
params.dr_com_des_W = [0; 0; 0];

params.rot_des = [0; 0; 0];
params.drot_des = [0; 0; 0];

params.R_pelvis_des = eye(3);
params.w_pelvis_des = [0; 0; 0];

f0 = [0; 0; params.g*params.m/4];
params.f_contacts_0_W = [f0; f0; f0; f0]; % initial guess for contract forces in world frame
