function tau = studentController(t, s, model, params)
% This function implements a Finite State Machine (FSM) for dynamic
% push recovery (stepping).

% --- FSM State ---
persistent robot_state;
persistent state_change;
% persistent swing_foot;
% persistent swing_trajectory; % This will store the struct from our generator
persistent qdes;
persistent traj;
persistent traj_idx;

% print(params)

% --- Initialize FSM on first run ---
if t == 0
    disp('initialize starting state')
    robot_state = 'stage_1'; % Start by standing on two feet
    state_change = true;
    traj_idx = 1;

    fprintf('State: %-10s | state_change: %d | Idx: %3d', ...
    robot_state, state_change, traj_idx);

end

% --- State vector components ---
q = s(1 : model.n);
dq = s(model.n+1 : 2*model.n);

% --- Get Key Robot States (You need this in all states) ---
[r_com, v_com] = computeComPosVel(q, dq, model);
[p1, p2, p3, p4] = computeFootPositions(q, model);
r_com_xy = r_com(1:2);
v_com_xy = v_com(1:2);

% --- Support Polygon (The 'box' your feet make) ---
poly_points_x = [p1(1), p2(1), p3(1), p4(1)];
poly_points_y = [p1(2), p2(2), p3(2), p4(2)];
k = convhull(poly_points_x, poly_points_y);
poly_x = poly_points_x(k);
poly_y = poly_points_y(k);

% disp('poly points')
% disp(poly_points_x)
% disp(poly_points_y)

% disp(t)

switch robot_state
    case 'stage_1'
        if state_change
            state_change = false;
            foot_des = params.stage1;
            qdes = solveFootIK(model, foot_des(:,1), foot_des(:,2), foot_des(:,3), foot_des(:,4), [0;0;-0.1], q);
            disp("changing to stage 1")
            disp('differences')
            traj = generate_trajectory(q, qdes, params.t1, 10);
            traj_idx = 1;
            % disp(traj)
            disp(norm(q - qdes))
            disp('qdes')
            disp(qdes)
        end
        %% Base controller to keep stance for now


        kp = 1800 ; 
        kd = 300 ;
        % x0 = getInitialState(model);
        % q0 = x0(1:model.n) ;
        q1 = traj(traj_idx,:).';
        tau = -kp*(q(model.actuated_idx)-q1(model.actuated_idx)) - kd*dq(model.actuated_idx) ;
        error_dist = norm(q(model.actuated_idx)-q1(model.actuated_idx));
    
        % Check if within tolerance
        if error_dist < params.tolerance
            traj_idx = traj_idx + 1;
            fprintf("Reached tolerance for traj idx %i\n", traj_idx)
        
        end

        if traj_idx == length(traj)
            robot_state = 'stage_end';
            state_change = true;
            disp(q)
            traj_idx = 1;
        end

    % case 'stage_2'
    %     if state_change
    %         state_change = false;
    %         foot_des = params.stage2;
    %         qdes = solveFootIK(model, foot_des(:,1), foot_des(:,2), foot_des(:,3), foot_des(:,4), q);
    %         disp("changing to stage 2")
    %         disp(norm(q - qdes))
    %     end
    %     %% Base controller to keep stance for now
    % 
    % 
    %     kp = 1300 ; 
    %     kd = 300 ;
    %     % x0 = getInitialState(model);
    %     % q0 = x0(1:model.n) ;
    %     q0 = qdes;
    %     tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx) ;
    case 'stage_end'
        if state_change
            state_change = false;
            qdes = q;
            % dqdes = zeros(size(dq));
            disp("changing to stage end")
            disp(norm(q - qdes))
        end
        kp = 3000 ; 
        kd = 300 ;
        q0 = qdes;
        tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx) ;

end


% =========================================================================
% --- MAIN STATE MACHINE ---
% =========================================================================
% 
% switch robot_state
% 
%     case 'DoubleSupport'
%         % --- 1. WE ARE STANDING ON TWO FEET ---
%         tau = calculate_double_support_torques(q, dq, model);
% 
%         % --- 2. CHECK IF WE NEED TO STEP (Capture Point Logic) ---
%         g = 9.81;
%         h_com = r_com(3); 
%         w0 = sqrt(g / h_com);
%         CP = r_com_xy + v_com_xy / w0;
% 
%         [in, on] = inpolygon(CP(1), CP(2), poly_x, poly_y);
% 
%         if ~(in || on)
%             % --- TRIGGER STEP ---
%             p_step_target_xy = CP; % This is 2x1 [x;y]
% 
%             % Decide which foot to use
%             if v_com_xy(2) > 0 % (Falling right)
%                 swing_foot = 'right';
%                 p_start_3d = (p3 + p4) / 2; % Avg pos of right foot (3x1)
%             else % (Falling left or forward/back)
%                 swing_foot = 'left';
%                 p_start_3d = (p1 + p2) / 2; % Avg pos of left foot (3x1)
%             end
% 
%             % Create a 3D target vector from the 2D CP
%             p_step_target_3d = [p_step_target_xy; p_start_3d(3)];
%             disp('p_step_target_3d')
%             disp(p_step_target_3d)
% 
%             % 3. Generate and save the trajectory plan
%             % swing_trajectory = generate_foot_trajectory(p_start_3d, p_step_target_3d, t);
% 
%             % 4. Change state
%             disp('Capture Point left polygon. Triggering step!');
%             robot_state = 'SwingLeg'; 
%         end
% 
% 
%     case 'SwingLeg'
%         % % --- WE ARE IN THE AIR, SWINGING ONE LEG ---
%         % 
%         % % 1. Get the desired position/velocity for the swing foot
%         % [p_desired, v_desired] = get_trajectory_point(swing_trajectory, t);
%         % 
%         % % 2. Calculate Stance Leg Torques (for balancing)
%         % tau_stance = calculate_single_support_torques(q, dq, model, swing_foot);
%         % 
%         % % 3. Calculate Swing Leg Torques (for foot-tracking)
%         % tau_swing = calculate_swing_leg_torques(q, dq, model, swing_foot, p_desired, v_desired);
%         % 
%         % % 4. Combine torques
%         % tau = tau_stance + tau_swing;
%         % 
%         % % 5. Check if the foot has landed
%         % if has_foot_landed(swing_trajectory, t)
%         %     disp('Step finished. Returning to Double Support.');
%         %     robot_state = 'DoubleSupport'; 
%         %     swing_foot = 'none';
%         % end
% 
%         kp = 1800 ; 
%         kd = 300 ;
%         x0 = getInitialState(model);
%         q0 = x0(1:model.n) ;
%         tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx) ;
% 
% 
% 
% end


% %% Base controller to keep stance for now
% kp = 1800 ; 
% kd = 300 ;
% % x0 = getInitialState(model);
% % q0 = x0(1:model.n) ;
% q0 = params.q1;
% tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx) ;

end


% =========================================================================
% --- HELPER FUNCTIONS ---
% =========================================================================

function tau = calculate_double_support_torques(q, dq, model)
    % This is your [Control #2.3] "soft" collaborative controller
    kp_posture = 500; kd_posture = 100; 
    x0 = getInitialState(model); q0 = x0(1:model.n) ;
    tau_posture = -kp_posture*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd_posture*dq(model.actuated_idx) ;
    [r_com, v_com] = computeComPosVel(q, dq, model);
    r_com_xy = r_com(1:2); v_com_xy = v_com(1:2);
    [p1, p2, p3, p4] = computeFootPositions(q, model);
    rd_com_x = (p1(1) + p2(1) + p3(1) + p4(1)) / 4;
    rd_com_y = (p1(2) + p2(2) + p3(2) + p4(2)) / 4;
    rd_com_xy = [rd_com_x; rd_com_y]; vd_com_xy = [0; 0];
    kp_com = 2000; kd_com = 300; 
    fd_xy = -kp_com * (r_com_xy - rd_com_xy) - kd_com * (v_com_xy - vd_com_xy);
    J_com = computeComJacobian(q, model); 
    J_com_xy_act = J_com(1:2, model.actuated_idx);
    tau_com = J_com_xy_act' * fd_xy;
    tau = tau_posture + tau_com;
end

function trajectory = generate_foot_trajectory(p_start, p_target, t_start)
    % Creates a 3D path for the foot.
    trajectory.p_start = p_start;   % 3D [x;y;z] start pos
    trajectory.p_target = p_target; % 3D [x;y;z] target pos
    trajectory.t_start = t_start;   % Time the step begins
    trajectory.duration = 0.4;      % How long the step takes (in seconds)
    trajectory.lift_height = 0.07;  % How high to lift the foot (7cm)
    
    disp('Generated new foot trajectory.');
end

function [p_d, v_d] = get_trajectory_point(trajectory, t_now)
    % Calculates the desired foot [x;y;z] position
    % and [vx;vy;vz] velocity for the current time t_now
    
    t_elapsed = t_now - trajectory.t_start;
    
    % 's' is the normalized time, from 0.0 to 1.0
    s = t_elapsed / trajectory.duration;
    s = max(0, min(1, s)); % Clamp s to be in [0, 1]
    
    if s >= 1.0
        % --- Step is finished, hold target position ---
        p_d = trajectory.p_target;
        v_d = [0; 0; 0];
    else
        % --- Step is in progress ---
        p_d_xy = (1 - s) * trajectory.p_start(1:2) + s * trajectory.p_target(1:2);
        v_d_xy = (trajectory.p_target(1:2) - trajectory.p_start(1:2)) ...
                 / trajectory.duration;
        p_d_z = trajectory.p_start(3) + trajectory.lift_height * sin(pi * s);
        v_d_z = (trajectory.lift_height * pi / trajectory.duration) * cos(pi * s);
        p_d = [p_d_xy; p_d_z];
        v_d = [v_d_xy; v_d_z];
    end
end

function tau_stance = calculate_single_support_torques(q, dq, model, swing_foot)
    % This function calculates torques for the STANCE leg to
    % balance the robot's COM over that single foot.

    [p1, p2, p3, p4] = computeFootPositions(q, model);

    if strcmp(swing_foot, 'right')
        stance_mask = [1; 1; 1; 1; 1; 0; 0; 0; 0; 0];
        rd_com_xy = (p1(1:2) + p2(1:2)) / 2; % Target is center of left foot
    elseif strcmp(swing_foot, 'left')
        stance_mask = [0; 0; 0; 0; 0; 1; 1; 1; 1; 1];
        rd_com_xy = (p3(1:2) + p4(1:2)) / 2; % Target is center of right foot
    else
        tau_stance = zeros(10,1);
        return;
    end

    [r_com, v_com] = computeComPosVel(q, dq, model);
    r_com_xy = r_com(1:2); v_com_xy = v_com(1:2); vd_com_xy = [0; 0];
    
    kp_posture = 500; kd_posture = 100; 
    x0 = getInitialState(model); q0 = x0(1:model.n) ;
    tau_posture_total = -kp_posture*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd_posture*dq(model.actuated_idx);

    kp_com = 3000; kd_com = 500; 
    fd_xy = -kp_com * (r_com_xy - rd_com_xy) - kd_com * (v_com_xy - vd_com_xy);

    J_com = computeComJacobian(q, model); 
    J_com_xy_act = J_com(1:2, model.actuated_idx);
    tau_com_total = J_com_xy_act' * fd_xy;

    tau_total = tau_posture_total + tau_com_total;
    tau_stance = tau_total .* stance_mask;
end


function tau_swing = calculate_swing_leg_torques(q, dq, model, swing_foot, p_desired, v_desired)
    % This function calculates torques for the SWING leg to
    % follow the desired foot trajectory (p_desired, v_desired).
    
    % Reconstruct the full state 's' (or 'x')
    s_full = [q; dq];
    
    % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    % --- FIX #1: Get dqy and G ---
    % Assume: dq is 20x1 (full), dqy is 20x1 (full), G is 20x16 (map from minimal to full)
    [~, dq_full_vector, G_full_to_minimal] = model.gamma_q(model, q, dq) ;
    % NOTE: We use dq_full_vector (20x1) for dqy below, as you specified its size.
    
    % We need the minimal velocity vector (dq_minimal) to multiply with the 16-column Jacobian (J_foot).
    % Use the pseudoinverse of G to get the 16x1 minimal velocity vector:
    % dq_minimal [16x1] = pinv(G) [16x20] * dq_full_vector [20x1]
    dq_minimal = pinv(G_full_to_minimal) * dq_full_vector;
    % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    
    % Get all foot positions and jacobians
    [p1, p2, p3, p4] = computeFootPositions(q, model);
    
    % These Jacobians are assumed to be [6 x 16] (Compatible with dq_minimal)
    [J1f_reduced, J1b_reduced, J2f_reduced, J2b_reduced] = computeFootJacobians(s_full, model); 
    
    if strcmp(swing_foot, 'right')
        swing_mask = [0; 0; 0; 0; 0; 1; 1; 1; 1; 1];
        p_actual = (p3 + p4) / 2;
        J_foot_full = (J2f_reduced + J2b_reduced) / 2; % Avg. 6D Jacobian [6 x 16]
        
    elseif strcmp(swing_foot, 'left')
        swing_mask = [1; 1; 1; 1; 1; 0; 0; 0; 0; 0];
        p_actual = (p1 + p2) / 2;
        J_foot_full = (J1f_reduced + J1b_reduced) / 2; % Avg. 6D Jacobian [6 x 16]
    else
        tau_swing = zeros(10,1);
        return;
    end
    
    % Select only the linear velocity rows (1-3)
    J_foot = J_foot_full(1:3, :); % J_foot is now [3 x 16]
    
    % --- 1. Get current foot velocity ---
    % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    % --- FIX #2: Use dq_minimal [16x1] with J_foot [3x16] ---
    % v_actual = [3x16] * [16x1] = [3x1]
    v_actual = J_foot * dq_minimal; 
    % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    
    % --- 2. Calculate 3D Task-Space Force (PD Controller) ---
    kp_foot = 750; 
    kd_foot = 100;
    
    p_error = p_desired - p_actual; 
    v_error = v_desired - v_actual;
    
    f_desired = kp_foot * p_error + kd_foot * v_error;
    
    % --- 3. Map 3D Force to Joint Torques ---
    % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    % --- FIX #3: Map back using G ---
    % Map f_desired [3x1] to generalized torques tau_y_minimal [16x1]
    % tau_y_minimal = [16x3] * [3x1] = [16x1]
    tau_y_minimal = J_foot' * f_desired;
    
    % Map minimal generalized torques tau_y_minimal [16x1] to full joint torques tau_q [20x1]
    % tau_q = [20x16] * [16x1] = [20x1]
    tau_task_all_joints = G_full_to_minimal * tau_y_minimal;
    % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    
    % Get the 10 actuated torques
    tau_task_actuated = tau_task_all_joints(model.actuated_idx);
    
    % Apply the mask to get torques *only* for the swing leg
    tau_task = tau_task_actuated .* swing_mask;
    
    % --- 4. Add "Null-Space" Posture Damping ---
    kp_posture_swing = 50; 
    kd_posture_swing = 10;
    
    x0 = getInitialState(model);
    q0 = x0(1:model.n) ;
    
    tau_posture_total = -kp_posture_swing * (q(model.actuated_idx) - q0(model.actuated_idx)) ...
                      - kd_posture_swing * dq(model.actuated_idx);
    
    tau_posture = tau_posture_total .* swing_mask;
    
    % --- 5. Combine and Finalize ---
    tau_swing = tau_task + tau_posture;
end

function landed = has_foot_landed(trajectory, t_now)
    % Checks if the step is finished
    landed = (t_now > (trajectory.t_start + trajectory.duration));
end