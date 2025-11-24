function tau = studentController(t, s, model, params)

    % --- PERSISTENT VARIABLES FOR STATE MACHINE ---
    persistent controller_mode      % 'QP' or 'STEPPING'
    persistent robot_state          % 'stage_1', 'stage_2', 'stage_end'
    persistent state_change
    persistent persist_params       % To store stepping targets/gains
    persistent qdes
    persistent traj
    persistent traj_idx
    persistent last_debug_bucket
    persistent direction
    persistent step_config          % Cache for geometric params (foot_dx, etc)

    % --- INITIALIZATION ---
    if isempty(controller_mode)
        controller_mode = 'ForceController';
        last_debug_bucket = -1;

        % We need to load the stepping parameters (foot_dx, tolerance) 
        % once at the start, as they might not be in the standard QP params
        step_config = studentParams(model); 
    end

    % Extract State
    q = s(1 : model.n);
    dq = s(model.n+1 : 2*model.n);

    % =========================================================================
    % MODE 1: QP FORCE CONTROLLER
    % =========================================================================
    if strcmp(controller_mode, 'ForceController')

        params = studentParams(model);

        % [Standard QP Setup Code]
        [p_CoM, v_CoM] = computeComPosVel(q, dq, model);
        yaw   = q(4);
        pitch = q(5);
        roll  = q(6);
        R_pelvis = rot_z(yaw) * rot_y(pitch) * rot_x(roll);
        w_pelvis = dq(4:6);

        p_CoM_des     = params.p_CoM_des;
        v_CoM_des     = params.v_CoM_des;
        a_CoM_des     = params.a_CoM_des;
        R_pelvis_des  = params.R_pelvis_des;
        w_pelvis_des  = params.w_pelvis_des;
        dw_pelvis_des = params.dw_pelvis_des;

        error_p_CoM = p_CoM - p_CoM_des;
        error_v_CoM = v_CoM - v_CoM_des;
        error_R_pelvis = 0.5 * vee_map(R_pelvis_des' * R_pelvis - R_pelvis' * R_pelvis_des);
        error_w_pelvis = w_pelvis - w_pelvis_des;

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

        A_eq = G_c;
        mu = params.mu;
        A_foot = [ 0,  0, -1; 1, 0, -mu; -1, 0, -mu; 0, 1, -mu; 0, -1, -mu];
        A_ineq = kron(eye(num_contacts), A_foot);
        b_ineq = zeros(size(A_ineq, 1), 1);
        H = 2 * eye(contact_forces_dim);
        f_obj = zeros(contact_forces_dim, 1);
        options = optimoptions('quadprog', 'Display', 'none', 'MaxIter', 1000);

        kp_CoM = params.kp_CoM;
        kd_CoM = params.kd_CoM;
        kp_pelvis = params.kp_pelvis;
        kd_pelvis = params.kd_pelvis;
        max_tries = 5;
        exitflag = -2;

        for i = 1:max_tries
            f_d = -kp_CoM .* error_p_CoM - kd_CoM .* error_v_CoM ...
                + [0; 0; params.mass * 9.81] + params.mass * a_CoM_des;
            tau_d = -kp_pelvis .* error_R_pelvis - kd_pelvis .* error_w_pelvis ...
                + params.I_pelvis * dw_pelvis_des;
            W_des = [f_d; tau_d];
            b_eq = W_des;
            [F_total, ~, exitflag] = quadprog(H, f_obj, A_ineq, b_ineq, A_eq, b_eq, [], [], [], options);
            if exitflag == 1
                break;
            end
            kp_CoM = kp_CoM * 0.8; kd_CoM = kd_CoM * 0.8;
            kp_pelvis = kp_pelvis * 0.8; kd_pelvis = kd_pelvis * 0.8;
        end

        % --- TRIGGER TRANSITION TO STEPPING ---
        if exitflag ~= 1
            warning('Force Control Failed at t=%.3f. Switching to STEP MODE.', t);

            % 1. Switch Mode
            controller_mode = 'STEPPING';
            state_change = true;
            robot_state = 'stage_1';

            % 2. Setup Step Parameters (Logic taken from your 'stage_0')
            persist_params = step_config; % Load struct with foot_dx/dy etc.

            % Determine direction based on current Velocity (more robust than acc)
            % If moving +X, step Left Leg (based on your logic), else Right.
            if v_CoM(1) > 0 
                direction = 'left';
                persist_params.stage1 = [[0.0921+persist_params.foot_dx;0.1305+persist_params.foot_dy;0.05],...
                                         [-0.0879+persist_params.foot_dx;0.1305+persist_params.foot_dy;0.05],...
                                         [0.0921;-0.1305;0],...
                                         [-0.0879;-0.1305;0]];
                persist_params.stage2 = [[0.0921+persist_params.foot_dx*2;0.1305+persist_params.foot_dy*2;0],...
                                        [-0.0879+persist_params.foot_dx*2;0.1305+persist_params.foot_dy*2;0],...
                                        [0.0921;-0.1305;0],...
                                        [-0.0879;-0.1305;0]];
                persist_params.mask = [0;0;1;1];
                persist_params.kp_left = 300;
                persist_params.kd_left = 30;
                persist_params.kp_right = 1300;
                persist_params.kd_right = 300;
            else
                direction = 'right';
                persist_params.stage1 = [[0.0921;0.1305;0],...
                                         [-0.0879;0.1305;0],...
                                         [0.0921+persist_params.foot_dx;-0.1305-persist_params.foot_dy;0.05],...
                                         [-0.0879+persist_params.foot_dx;-0.1305-persist_params.foot_dy;0.05]];
                persist_params.stage2 = [[0.0921;0.1305;0],...
                                        [-0.0879;0.1305;0],...
                                        [0.0921+persist_params.foot_dx*2;-0.1305-persist_params.foot_dy*2;0],...
                                        [-0.0879+persist_params.foot_dx*2;-0.1305-persist_params.foot_dy*2;0]];
                persist_params.mask = [1;1;0;0];
                persist_params.kp_left = 1300;
                persist_params.kd_left = 300;
                persist_params.kp_right = 300;
                persist_params.kd_right = 30;
            end

            % Return a holding torque for this specific tick to avoid error
            % kp = 1800; kd = 300;
            % tau = -kp*dq(model.actuated_idx); 
            tau = studentController(t, s, model, params);
            return;
        end

        % Jacobians
        [J1f_w, J1b_w, J2f_w, J2b_w] = computeFootJacobians(s, model);
        J_feet_world_linear = {J1f_w(4:6,:), J1b_w(4:6,:), J2f_w(4:6,:), J2b_w(4:6,:)};

        % Torque Calculation
        tau_dy = zeros(length(model.independent_idx), 1);
        for i = 1:num_contacts
            f_i_world = F_total((i-1)*3+1 : i*3);
            J_i_world_linear = J_feet_world_linear{i};
            tau_dy = tau_dy + J_i_world_linear' * f_i_world;
        end

        tau_full = zeros(model.n, 1);
        tau_full(model.independent_idx) = -tau_dy;
        tau_model = tau_full(model.actuated_idx);
        % Joint Damping
        dq_actuated = dq(model.actuated_idx);
        tau_damping = -params.kd_internal .* dq_actuated;
        tau = tau_model + tau_damping;

    % =========================================================================
    % MODE 2: STEPPING FSM CONTROLLER
    % =========================================================================
    elseif strcmp(controller_mode, 'STEPPING')

        debug_interval = 0.05;

        switch robot_state
            case 'stage_1' % Swing Leg Up
            if state_change
                state_change = false;
                foot_des = persist_params.stage1;
                qdes = solveFootIK(model, foot_des(:,1), foot_des(:,2), foot_des(:,3), foot_des(:,4), persist_params.mask, q);
                disp("changing to stage 1")
                % disp('differences')
                traj = generate_trajectory(q(model.actuated_idx), qdes(model.actuated_idx), 2);
                traj_idx = 1;
                % % disp(traj)
                % disp(norm(q - qdes))
                % disp('qdes')
                % disp(qdes)
            end
            % Base controller to keep stance for now

            % left leg
            kp_left = persist_params.kp_left;
            kd_left = persist_params.kd_left;
            kp_right = persist_params.kp_right;
            kd_right = persist_params.kd_right;
            % left leg
            kp_arr = [kp_left; kp_right; kp_left; kp_right; kp_left; kp_right; kp_left; kp_right; kp_left*0.5; kp_right*0.5];
            kd_arr = [kd_left; kd_right; kd_left; kd_right; kd_left; kd_right; kd_left; kd_right; kd_left*0.5; kd_right*0.5];

            % left leg
            kp_arr = ones([5,2]);
            kp_arr(1:4,1) = kp_arr(1:4,1) .* 500;
            kp_arr(5,1) = kp_arr(5,1) .* 500;
            kp_arr(:,2) = kp_arr(:,2) * 1800;
            kp_arr = [kp_arr(:,1) ; kp_arr(:,2)];

            kd_arr = ones([10,1]) .* 100;

            q1 = traj(traj_idx,:).';
            % q1 = qdes(model.actuated_idx);
            % tau = -kp_one.*(q(model.actuated_idx)-q1(model.actuated_idx)) - kd*dq(model.actuated_idx) ;
            tau = -kp_arr.*(q(model.actuated_idx)-q1) - kd_arr.*dq(model.actuated_idx) ;

            error_dist = norm((q(model.actuated_idx)-q1))^2;

            current_bucket = floor(t / debug_interval);

            % If we have moved to a new bucket, print and update
            if current_bucket > last_debug_bucket
                fprintf('[DEBUG t=%.2f] Error Distance: %d | Traj Idx: %i\n', t, error_dist, traj_idx);
                last_debug_bucket = current_bucket;
            end

            % Check if within tolerance
            if error_dist < persist_params.tolerance
                fprintf("Reached tolerance for traj idx %i at %d\n", traj_idx, t)
                traj_idx = traj_idx + 1;
                if traj_idx > height(traj)
                    robot_state = 'stage_2';
                    state_change = true;
                    disp(q)
                end
            end

            case 'stage_2'
            if state_change
                state_change = false;
                foot_des = persist_params.stage2;
                qdes = solveFootIK(model, foot_des(:,1), foot_des(:,2), foot_des(:,3), foot_des(:,4), persist_params.mask, q);
                disp("changing to stage 2")
                traj = generate_trajectory(q(model.actuated_idx), qdes(model.actuated_idx), 2);
                traj_idx = 1;
            end
            % Base controller to keep stance for now
            kp_left = persist_params.kp_left;
            kd_left = persist_params.kd_left;
            kp_right = persist_params.kp_right;
            kd_right = persist_params.kd_right;
            % left leg
            kp_arr = [kp_left; kp_right; kp_left; kp_right; kp_left; kp_right; kp_left; kp_right; kp_left*0.5; kp_right*0.5];

            kd_arr = [kd_left; kd_right; kd_left; kd_right; kd_left; kd_right; kd_left; kd_right; kd_left*0.5; kd_right*0.5];

            q1 = traj(traj_idx,:).';
            tau = -kp_arr.*(q(model.actuated_idx)-q1) - kd_arr.*dq(model.actuated_idx) ;

            error_dist = norm((q(model.actuated_idx)-q1))^2;

            current_bucket = floor(t / debug_interval);

            % If we have moved to a new bucket, print and update
            if current_bucket > last_debug_bucket
                fprintf('[DEBUG t=%.2f] Error Distance: %d | Traj Idx: %i\n', t, error_dist, traj_idx);
                last_debug_bucket = current_bucket;
            end

            [p1_curr, p2_curr, p3_curr, p4_curr] = computeFootPositions(q, model);
            p_foot = [p1_curr, p2_curr, p3_curr, p4_curr];
            if strcmp(direction,'left')
                p_foot = p_foot(:,1:2);
            else
                p_foot = p_foot(:,3:4);
            end

            if min(p_foot(3,:)) < 1e-3
                disp('touchdown')
                robot_state = 'stage_end';
                state_change = true;
                disp(q)
            end

            % Check if within tolerance
            if error_dist < persist_params.tolerance
                fprintf("Reached tolerance for traj idx %i at %d\n", traj_idx, t)
                traj_idx = traj_idx + 1;

            end

        case 'stage_end'
            if state_change
                disp(t)
                state_change = false;
                clear functions;
                qdes = q;
                % dqdes = zeros(size(dq));
                disp("changing to stage end")
                % disp(norm(q - qdes))
            end
            kp = 1800 ;
            kd = 300 ;
            q0 = qdes;
            tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx) ;
        end
    end
end

% =========================================================================
% HELPER FUNCTIONS
% =========================================================================
function q_sol = solveFootIK(model, p1_tgt, p2_tgt, p3_tgt, p4_tgt, feet_active, q0)
% SOLVEFOOTIK Solves for joint configuration q.
%   Uses 'fmincon' to STRICTLY enforce active foot positions as constraints,
%   while minimizing CoM error as the objective.
%
%   INPUTS:
%       model       - Robot model structure.
%       pX_tgt      - Target position for feet.
%       feet_active - 4x1 Binary. 1 = STRICT CONSTRAINT. 0 = IGNORED/FREE.
%       com_delta   - Desired CoM shift.
%       com_weight  - Importance of CoM in the objective.
%       q0          - Initial guess.

    % 1. Check input arguments
    % if nargin < 8
    %     error('Please provide all arguments including feet_active and q0.');
    % end
    % if ~isfield(model, 'actuated_idx')
    %     error('The model structure must contain "actuated_idx".');
    % end

    min_z = min([p1_tgt(3), p2_tgt(3), p3_tgt(3), p4_tgt(3)]);

    if min_z < 0

        p1_tgt(3) = p1_tgt(3) - min_z;
        p2_tgt(3) = p2_tgt(3) - min_z;
        p3_tgt(3) = p3_tgt(3) - min_z;
        p4_tgt(3) = p4_tgt(3) - min_z;

        q0(3) = q0(3) - min_z;

    end

    % 2. Prepare Optimization Variables
    act_indices = model.actuated_idx;
    x0 = q0(act_indices); 
    
    active_flags = feet_active(:);

    % 4. Calculate Absolute CoM Target

    % 5. Define Solver Options for 'fmincon'
    % 'sqp' is excellent for handling equality constraints in kinematics
    options = optimoptions('fmincon', ...
        'Algorithm', 'sqp', ... 
        'Display', 'off', ...
        'MaxIterations', 1000, ...
        'ConstraintTolerance', 1e-4, ... % STRICT enforcement (0.1mm)
        'StepTolerance', 1e-6);

    % 6. Objective Function (Scalar)
    % Minimizes CoM Error (weighted)
    fun = @(x) objectiveFunction(x, act_indices, q0, model, p1_tgt, p2_tgt, p3_tgt, p4_tgt, active_flags);

    % 7. Constraint Function (Non-linear Equalities)
    % Enforces Foot Position = Target strictly
    nonlcon = @(x) constraintFunction(x, act_indices, q0, model, p1_tgt, p2_tgt, p3_tgt, p4_tgt, active_flags);

    % 8. Run the Solver
    % fmincon(fun, x0, A, b, Aeq, beq, lb, ub, nonlcon, options)
    lb = []; ub = []; 
    [x_sol, fval, exitflag, output] = fmincon(fun, x0, [], [], [], [], lb, ub, nonlcon, options);

    % --- Print Results ---
    % fprintf('IK Solver (fmincon) Objective Cost: %e\n', fval);
    % fprintf('  Max Constraint Violation: %e (Should be close to 0)\n', output.constrviolation);

    % 9. Reconstruct Solution
    q_sol = q0;                 
    q_sol(act_indices) = x_sol; 

    % 10. Convergence Check
    if exitflag <= 0
        warning('IK Solver did not converge perfectly.');
    end
end

function cost = objectiveFunction(x, indices, q_template, model, p1_tgt, p2_tgt, p3_tgt, p4_tgt, active)
    % OBJECTIVE: Minimize Weighted CoM Error
    
    % Reconstruct q
    q_current = q_template;
    q_current(indices) = x;

    % Forward Kinematics Feet
    [p1_curr, p2_curr, p3_curr, p4_curr] = computeFootPositions(q_current, model);

    
    % For every ACTIVE foot, append its XYZ error to ceq.
    % The solver must drive these values to 0.
    diff1 = (p1_curr - p1_tgt) * (~active(1));
    diff2 = (p2_curr - p2_tgt) * (~active(2));
    diff3 = (p3_curr - p3_tgt) * (~active(3));
    diff4 = (p4_curr - p4_tgt) * (~active(4));
    
    % % Weighted Error Vector
    % diff_com = (r_com_curr - com_t)*10; 
    err_vec = [diff1; diff2; diff3; diff4];
    % Return Scalar Sum of Squares
    cost = sum(err_vec.^2);

end

function [c, ceq] = constraintFunction(x, indices, q_template, model, p1_t, p2_t, p3_t, p4_t, active)
    % CONSTRAINTS: Enforce Foot Positions
    
    % Reconstruct q
    q_current = q_template;
    q_current(indices) = x;
    
    % Forward Kinematics Feet
    [p1, p2, p3, p4] = computeFootPositions(q_current, model);
    
    % Inequality Constraints (None)
    c = [];
    
    % Equality Constraints (Strict)
    ceq = [];
    
    % For every ACTIVE foot, append its XYZ error to ceq.
    % The solver must drive these values to 0.
    if active(1), ceq = [ceq; p1 - p1_t]; end
    if active(2), ceq = [ceq; p2 - p2_t]; end
    if active(3), ceq = [ceq; p3 - p3_t]; end
    if active(4), ceq = [ceq; p4 - p4_t]; end
end

function [q, t] = generate_trajectory(q_start, q_des, num_steps)
% GENERATE_TRAJECTORY Generates a minimum-jerk (quintic) trajectory positions.
%
% Inputs:
%   q_start   - Initial joint configuration (Nx1 or 1xN vector)
%   q_des     - Desired/Final joint configuration (Nx1 or 1xN vector)
%   delta_t   - Total duration of movement (seconds)
%   num_steps - Number of discrete time steps
%
% Outputs:
%   q         - Joint Positions (num_steps x N)
%   t         - Time vector (num_steps x 1)

    % Ensure inputs are column vectors for consistent math
    q0 = q_start(:);
    qf = q_des(:);
    
    % % Create time vector
    % t = linspace(0, delta_t, num_steps)';
    % 
    % % Normalize time to range [0, 1] to simplify polynomial calculation
    % % tau = t / T
    % tau = t / delta_t;
    t_steps = linspace(0, 1, num_steps)';
    
    % --- QUADRATIC EASE-OUT ---
    % This is much simpler than quintic.
    % It starts at max speed and decelerates to 0 velocity at the end.
    s = 1 - (1 - t_steps).^2;
    
    % Calculate Trajectories for all joints
    % q(t) = q0 + (qf - q0) * s(tau)
    delta_q = (qf - q0)'; % Transpose to row vector for broadcasting
    
    q = repmat(q0', num_steps, 1) + (s * delta_q);

    figure(5);
    plot(t_steps, q, 'LineWidth', 2);
    grid on;
    
    % Labeling
    xlabel('Time (s)');
    ylabel('Position');
    title('Minimum Jerk Trajectory');
    
    % Dynamic Legend Generation
    num_joints = length(q0);
    legend_labels = cell(1, num_joints);
    for i = 1:num_joints
        legend_labels{i} = sprintf('Joint %d', i);
    end
    legend(legend_labels, 'Location', 'best');

end