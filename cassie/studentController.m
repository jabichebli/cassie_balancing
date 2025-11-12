function tau = studentController(t, s, model, ~)
% Modify this code to calculate the joint torques
% t - time
% s - state of the robot
% model - struct containing robot properties
% params - user defined parameters in studentParams.m
% tau - 10x1 vector of joint torques

% State vector components ID
q = s(1 : model.n);
dq = s(model.n+1 : 2*model.n);

%% [Control #1] zero control
% tau = zeros(10,1);

%% [Control #2] Highest Score so far on leaderboard 

% Leaderboard score  |  Kp  |  Kd
%      218.64        | 1800 |  300
%      218.37        | 3000 |  500
%      163.67        | 1200 |  200 

% kp = 1800 ; 
% kd = 300 ;
% x0 = getInitialState(model);
% q0 = x0(1:model.n) ;
% tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx) ;

%% [Control #3] Testing Basic Controller
kp = 1800 ; % second
kd = 300 ; % second
x0 = getInitialState(model);
q0 = x0(1:model.n) ;
tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx) ;

%% [Control #4]  Test 2 
% Define params as persistent so it is initialized only ONCE
% persistent params;
%% --- Controller Setup (Runs only on first call) ---
% if isempty(params)
%     fprintf('Initializing controller parameters...\n');
%     % Initialize an empty params struct
%     params = struct;
%     % --- Define Desired States ---
% 
%     % 1. Desired Joint Positions (from initial state)
%     x0 = getInitialState(model);
%     q0 = x0(1:model.n);
%     % q0(model.actuated_idx) is a [1, 10] row vector.
%     params.q_des_joints = q0(model.actuated_idx); 
% 
%     % 2. Desired COM Height
%     params.z_com_des = 0.8894; % [m]
% 
%     % --- Define Controller Gains ---
% 
%     % --- Layer 1: Joint-Space PD (YOUR WORKING CONTROLLER) ---
%     % We use your proven high-gain values as our base "stiffness"
%     params.Kp_joints = ones(10, 1) * 1800; % 1200  %800
%     params.Kd_joints = ones(10, 1) * 300; % 200   %250
% 
%     % --- Layer 2: Torso Orientation (SMART NUDGE) ---
%     % We use smaller gains for the "smart" correction, so it
%     % "nudges" the stiff controller instead of fighting it.
%     params.Kp_roll = 0; % 200
%     params.Kd_roll = 0;  % 50 
% 
%     params.Kp_pitch = 0; % Keep off for now
%     params.Kd_pitch = 0; % Keep off for now
% 
%     % --- Layer 3: COM Height (Anti-Collapsing) ---
%     % Start with this off. We can tune it later if needed.
%     params.Kp_z = 0;
%     params.Kd_z = 0;
% end
% %% --- Controller Logic (Runs every time step) ---
% % State vector components ID
% q = s(1 : model.n);
% dq = s(model.n+1 : 2*model.n);
% 
% %% --- Layer 1: Joint-Space PD (The "Stiff" Controller) ---
% % This is your working Control #2
% q_act = q(model.actuated_idx);
% dq_act = dq(model.actuated_idx);
% q_des = params.q_des_joints;
% dq_des = zeros(10, 1);
% e_pos = q_des - q_act;
% e_vel = dq_des - dq_act;
% tau_pd = params.Kp_joints .* e_pos + params.Kd_joints .* e_vel;
% 
% %% --- Layer 2: Torso Orientation PD (The "Smart Nudge") ---
% q_roll = q(6);
% dq_roll = dq(6);
% e_roll = 0 - q_roll;
% e_dot_roll = 0 - dq_roll;
% tau_roll_correction = params.Kp_roll * e_roll + params.Kd_roll * e_dot_roll;
% 
% q_pitch = q(5);
% dq_pitch = dq(5);
% e_pitch = 0 - q_pitch;
% e_dot_pitch = 0 - dq_pitch;
% tau_pitch_correction = params.Kp_pitch * e_pitch + params.Kd_pitch * e_dot_pitch;
% 
% tau_balance = zeros(10, 1);
% % Roll (sideways) correction:
% tau_balance(1) = -tau_roll_correction; % Left hip abduction
% tau_balance(2) = tau_roll_correction; % Right hip abduction
% % Pitch (front/back) correction:
% tau_balance(5) = tau_pitch_correction; % Left hip flexion
% tau_balance(6) = tau_pitch_correction; % Right hip flexion
% 
% %% --- Layer 3: COM Height PD (Anti-Collapsing) ---
% [r_com, v_com] = computeComPosVel(q, dq, model);
% z_com_curr = r_com(3);
% dz_com_curr = v_com(3);
% z_com_des = params.z_com_des;
% e_z = z_com_des - z_com_curr;
% e_dot_z = 0 - dz_com_curr;
% F_z_correction = params.Kp_z * e_z + params.Kd_z * e_dot_z;
% 
% tau_height = zeros(10, 1); 
% tau_height(5) = -F_z_correction; % Left hip flexion
% tau_height(6) = -F_z_correction; % Right hip flexion
% tau_height(7) = F_z_correction; % Left knee
% tau_height(8) = F_z_correction; % Right knee
% 
% %% --- Final Torque Calculation ---
% % The final torque is your stiff controller + the smart nudge
% tau = tau_pd + tau_balance + tau_height;
% 

end