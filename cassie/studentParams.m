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

params.Kp_f = 2000;
params.Kd_f = 300;

params.Kp_tau = 1000;
params.Kd_tau = 1;

params.r_com_des_W = r_com_0;
params.dr_com_des_W = [0; 0; 0];

params.rot_des = [0; 0; 0];
params.drot_des = [0; 0; 0];

f0 = [0; 0; params.g*params.m/4];
params.f_contacts_0_W = [f0; f0; f0; f0]; % initial guess for contract forces in world frame
