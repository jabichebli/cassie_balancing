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

persistent last_debug_bucket;
persistent persist_params;
persistent direction;

persistent last_t;

% print(params)

debug_interval = 0.05;


% --- Initialize FSM on first run ---
if isempty(robot_state)
    disp('initialize starting state')
    last_t = 0;
    persist_params = studentParams(model);
    
    robot_state = 'stage_0'; % Start by standing on two feet
    state_change = true;
    traj_idx = 1;
    last_debug_bucket = -1;

    fprintf('State: %-10s | state_change: %d | Idx: %i\n', ...
    robot_state, state_change, traj_idx);

end

% --- State vector components ---
q = s(1 : model.n);
dq = s(model.n+1 : 2*model.n);

% --- Get Key Robot States (You need this in all states) ---
% [r_com, v_com] = computeComPosVel(q, dq, model);
% [p1, p2, p3, p4] = computeFootPositions(q, model);
% r_com_xy = r_com(1:2);
% v_com_xy = v_com(1:2);

% --- Support Polygon (The 'box' your feet make) ---
% poly_points_x = [p1(1), p2(1), p3(1), p4(1)];
% poly_points_y = [p1(2), p2(2), p3(2), p4(2)];
% k = convhull(poly_points_x, poly_points_y);
% poly_x = poly_points_x(k);
% poly_y = poly_points_y(k);

% disp('poly points')
% disp(poly_points_x)
% disp(poly_points_y)

% disp(t)

switch robot_state
    case 'stage_0'
        if state_change
            state_change = false;
        end
        kp = 1800 ; 
        kd = 300 ;

        x0 = getInitialState(model);
        q0 = x0(1:model.n);
        tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx) ;

        a_com = [0;0;0];
        if t>0.01
            [r_com, v_com] = computeComPosVel(q, dq, model);
            a_com = v_com/t;
        end


        if norm(a_com(1:2)) > 0.01
        % if true
            state_change = true;
            robot_state = 'stage_1';
            if a_com(1) > 0
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
        end

    case 'stage_1'
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
        %% Base controller to keep stance for now

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
        %% Base controller to keep stance for now
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