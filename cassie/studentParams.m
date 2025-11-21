function params = studentParams(model)
% define any parameters here 
% params - struct

params = struct();


params.m  = model.M;
params.g = 9.81;

params.Kp_f = 50;
params.Kd_f = 5;

params.Kp_tau = 50;
params.Kd_tau = 5;

params.r_com_des_W = [0; 0; 1.006];
params.dr_com_des_W = [0; 0; 0];

params.rot_des = [0; 0; 0];
params.drot_des = [0; 0; 0];

f0 = [0; 0; params.g*params.m/4];
params.f_contacts_0_W = [f0; f0; f0; f0]; % initial guess for contract forces in world frame
