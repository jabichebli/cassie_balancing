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
persistent last_ik_bucket;

persistent persist_params;
persistent direction;

persistent last_q1;

persistent last_t;
persistent t_stage_start;

persistent hip_abductor;

% print(params)

debug_interval = 0.1;
ik_interval = 0.05;


% --- Initialize FSM on first run ---
if isempty(robot_state)
    disp('initialize starting state')
    last_t = 0;
    persist_params = studentParams(model);
    
    robot_state = 'stage_0'; % Start by standing on two feet
    state_change = true;
    traj_idx = 1;
    last_debug_bucket = -1;
    last_ik_bucket = -1;

    fprintf('State: %-10s | state_change: %d | Idx: %i\n', ...
    robot_state, state_change, traj_idx);

end

% --- State vector components ---
q = s(1 : model.n);
dq = s(model.n+1 : 2*model.n);

% p_target_ankle = [-0.2052;-0.6278;0.1047+0.05];
% p_target_toe = [-0.3850;-0.6244;0.1106+0.05];

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


        % if norm(a_com(1:2)) > 0.01
        if true
            state_change = true;
            robot_state = 'stage_1';
            % if a_com(1) > 0
            if false
                % direction = 'left';
                % persist_params.stage1 = [[0.0921+persist_params.foot_dx;0.1305+persist_params.foot_dy;0.05],...
                %                          [-0.0879+persist_params.foot_dx;0.1305+persist_params.foot_dy;0.05],...
                %                          [0.0921;-0.1305;0],...
                %                          [-0.0879;-0.1305;0]];
                % persist_params.stage2 = [[0.0921+persist_params.foot_dx*2;0.1305+persist_params.foot_dy*2;0],...
                %                         [-0.0879+persist_params.foot_dx*2;0.1305+persist_params.foot_dy*2;0],...
                %                         [0.0921;-0.1305;0],...
                %                         [-0.0879;-0.1305;0]];
                % persist_params.mask = [0;0;1;1];
                % persist_params.stage1_kp_left = 300;
                % persist_params.stage1_kd_left = 30;
                % persist_params.stage1_kp_right = 500;
                % persist_params.stage1_kd_right = 80;
                % 
                % persist_params.stage2_kp_left = 300;
                % persist_params.stage2_kd_left = 30;
                % persist_params.stage2_kp_right = 1300;
                % persist_params.stage2_kd_right = 300;
                % 
                % kp_left = 300;
                % kd_left = 30;
                % kp_right = 1300;
                % kd_right = 300;
                % 
                % persist_params.kp_swing = [kp_left; kp_right;
                %                            kp_left; kp_right;
                %                            kp_left; kp_right;
                %                            kp_left; kp_right;
                %                            kp_left; kp_right];
                % persist_params.kp_swing = [kd_left; kd_right;
                %                            kd_left; kd_right;
                %                            kd_left; kd_right;
                %                            kd_left; kd_right;
                %                            kd_left; kd_right];
        
            else
                % we're stepping with the right foot
                direction = 'right';
                %left toe, left ankle, right toe, right ankle
                % persist_params.stage1 = [[0.0921;0.1305;-0.05],...
                %                          [-0.0879;0.1305;-0.05],...
                persist_params.stage1 = [[0.0921;0.1305+persist_params.foot_dy*1.2;-0.3],...
                                         [-0.0879;0.1305+persist_params.foot_dy*1.2;-0.3],...
                                         [0.0921+persist_params.foot_dx;-0.1305-persist_params.foot_dy;0.3],...
                                         [-0.0879+persist_params.foot_dx;-0.1305-persist_params.foot_dy;0.3]];
                persist_params.p_target_ankle = [0.0921+persist_params.foot_dx;-0.1305-persist_params.foot_dy;0.3];
                persist_params.p_target_toe = [-0.0879+persist_params.foot_dx;-0.1305-persist_params.foot_dy;0.3];
                
                persist_params.stage2 = [[0.0921-persist_params.foot_dx*2;0.1305;0],...
                                        [-0.0879-persist_params.foot_dx*2;0.1305;0],...
                                        [0.0921+persist_params.foot_dx*2;-0.1305-persist_params.foot_dy*3;0],...
                                        [-0.0879+persist_params.foot_dx*2;-0.1305-persist_params.foot_dy*3;0]];
                persist_params.mask = [1;1;0;0];
                % persist_params.kp_left = 1300;
                % persist_params.kd_left = 300;
                % persist_params.kp_right = 300;
                % persist_params.kd_right = 30;

                persist_params.stage1_kp_left = 1400;
                persist_params.stage1_kd_left = 300;
                persist_params.stage1_kp_right = 1700;
                persist_params.stage1_kd_right = 300;


                % persist_params.stage2_kp_right = 300;
                % persist_params.stage2_kd_right = 30;
                % persist_params.stage2_kp_left = 1300;
                % persist_params.stage2_kd_left = 300;
                % 
                % kp_left = 500;
                % kd_left = 50;
                % kp_right = 1500;
                % kd_right = 300;
                % 
                % persist_params.kp_swing = [kp_left; kp_right;
                %                            kp_left; kp_right;
                %                            kp_left; kp_right;
                %                            kp_left; kp_right;
                %                            kp_left; kp_right];
                % persist_params.kp_swing = [kd_left; kd_right;
                %                            kd_left; kd_right;
                %                            kd_left; kd_right;
                %                            kd_left; kd_right;
                %                            kd_left; kd_right];
            end
        end

    case 'stage_1'
        if state_change
            state_change = false;
            disp("changing to stage 1")

            % foot_des = persist_params.stage1;
            % qdes_tmp = solveFootIK(model, foot_des(:,1), foot_des(:,2), foot_des(:,3), foot_des(:,4), persist_params.mask, q);
            % q_tmp = solveFootIK(model, foot_des(:,1), foot_des(:,2), foot_des(:,3), foot_des(:,4), [1;1;0;0], q);
            % % q1 = solveSingleFootIK(model, 'world', 'left', foot_des(:,1), foot_des(:,2), q1);
            q_tmp = solveSingleFootIK(model, 'body', 'right', persist_params.p_target_ankle, persist_params.p_target_toe, q);
            hip_abductor = q_tmp(8);
            % 
            % 
            % traj = generate_trajectory(q(model.actuated_idx), q1(model.actuated_idx), 2);
            % traj_idx = 1;
        end
        current_bucket = floor(t / ik_interval);

        if current_bucket > last_ik_bucket
            foot_des = persist_params.stage1;
            q1 = q;
            last_q1 = q1;

            q1 = solveFootIK(model, foot_des(:,1), foot_des(:,2), foot_des(:,3), foot_des(:,4), [1;1;0;0], q);

            % q1 = solveSingleFootIK(model, 'world', 'left', foot_des(:,1), foot_des(:,2), q1);
            q2 = solveSingleFootIK(model, 'body', 'right', persist_params.p_target_ankle, persist_params.p_target_toe, q);
            q1([8, 10, 12, 14, 20]) = q2([8, 10, 12, 14, 20]);
            % q1(7) = -hip_abductor * (1 - exp(-t / 0.1));

            % q1(9) = q1(9) - 0.3 * (1 - exp(-t / 0.13));
            % q1(8) = q1(8) + deg2rad(15 * (1 - exp(-t / 0.5)))/2;

            fprintf('[DEBUG Current Bucket=%i]\n', current_bucket);

            traj = generate_trajectory(q(model.actuated_idx), q1(model.actuated_idx), 2);
            traj_idx = 1;

            last_ik_bucket = current_bucket;
        end

        %% Base controller to keep stance for now

        kp_left = persist_params.stage1_kp_left;
        kd_left = persist_params.stage1_kd_left;
        kp_right = persist_params.stage1_kp_right;
        kd_right = persist_params.stage1_kd_right;

        % weights

        % hip abduction, hip rotation, hip flexion, knee joint, toe joint
        left_weights = [1, 1, 0.1, 0.2, 0.1];
        right_weights = left_weights;
        % right_weights = [1, 1, 0.2, 0.2, 0.1];

        C = [left_weights(:),right_weights(:)].';   %'
        total_weights = C(:);

        kp_arr = total_weights.*[kp_left; kp_right; kp_left; kp_right; kp_left; kp_right; kp_left; kp_right; kp_left; kp_right];
        kd_arr = [kd_left; kd_right; kd_left; kd_right; kd_left; kd_right; kd_left; kd_right; kd_left; kd_right];

        q1 = traj(traj_idx,:).';
        tau = -kp_arr.*(q(model.actuated_idx)-q1) - kd_arr.*dq(model.actuated_idx) ;

        error_dist = norm((q(model.actuated_idx)-q1))^2;
    


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

        if t > 0.5 && min(p_foot(3,:)) < 1e-3 
            disp('touchdown')
            robot_state = 'stage_end';
            state_change = true;
            disp(q)
        end

        % Check if within tolerance
        if error_dist < persist_params.tolerance
            fprintf("Reached tolerance for traj idx %i at %d\n", traj_idx, t)
            % [p1_curr, p2_curr, p3_curr, p4_curr] = computeFootPositions(q, model)
            traj_idx = traj_idx + 1;
            if traj_idx > height(traj)
                robot_state = 'stage_end';
                state_change = true;
            end
        end

    % case 'stage_2'
    %     if state_change
    %         state_change = false;
    %         foot_des = persist_params.stage2;
    %         qdes = solveFootIK(model, foot_des(:,1), foot_des(:,2), foot_des(:,3), foot_des(:,4), persist_params.mask, q);
    %         disp("changing to stage 2")
    %         traj = generate_trajectory(q(model.actuated_idx), qdes(model.actuated_idx), 2);
    %         traj_idx = 1;
    %     end
    %     %% Base controller to keep stance for now
    %     kp_left = persist_params.stage2_kp_left;
    %     kd_left = persist_params.stage2_kd_left;
    %     kp_right = persist_params.stage2_kp_right;
    %     kd_right = persist_params.stage2_kd_right;
    %     % left leg
    %     kp_arr = [kp_left; kp_right; kp_left; kp_right; kp_left; kp_right; kp_left; kp_right; kp_left*0.5; kp_right*0.5];
    % 
    %     kd_arr = [kd_left; kd_right; kd_left; kd_right; kd_left; kd_right; kd_left; kd_right; kd_left*0.5; kd_right*0.5];
    % 
    %     q1 = traj(traj_idx,:).';
    %     tau = -kp_arr.*(q(model.actuated_idx)-q1) - kd_arr.*dq(model.actuated_idx) ;
    % 
    %     error_dist = norm((q(model.actuated_idx)-q1))^2;
    % 
    %     current_bucket = floor(t / debug_interval);
    % 
    %     % If we have moved to a new bucket, print and update
    %     if current_bucket > last_debug_bucket
    %         fprintf('[DEBUG t=%.2f] Error Distance: %d | Traj Idx: %i\n', t, error_dist, traj_idx);
    %         last_debug_bucket = current_bucket;
    %     end
    % 
    %     [p1_curr, p2_curr, p3_curr, p4_curr] = computeFootPositions(q, model);
    %     p_foot = [p1_curr, p2_curr, p3_curr, p4_curr];
    %     if strcmp(direction,'left')
    %         p_foot = p_foot(:,1:2);
    %     else
    %         p_foot = p_foot(:,3:4);
    %     end
    % 
    %     if min(p_foot(3,:)) < 1e-3
    %         disp('touchdown')
    %         robot_state = 'stage_end';
    %         state_change = true;
    %         disp(q)
    %     end
    % 
    %     % Check if within tolerance
    %     if error_dist < persist_params.tolerance
    %         fprintf("Stage %i: Reached tolerance for traj idx %i at %d\n", traj_idx, t)
    %         traj_idx = traj_idx + 1;
    % 
    %     end

    case 'stage_end'
        if state_change
            disp(t)
            state_change = false;
            clear functions;
            qdes = q;
            t_stage_start = t;
            % dqdes = zeros(size(dq));
            disp("changing to stage end, holding position")
            % disp(norm(q - qdes))
        end
        
        kp = 1800 ;
        kd = 300 ;

        left_weights = [1, 1, 0.7, 0.6, 0.4];
        right_weights = left_weights;
        % right_weights = [1, 1, 0.2, 0.2, 0.1];

        C = [left_weights(:),right_weights(:)].';   %'
        total_weights = C(:);
        
        damping_modifier = 0.3 + 0.7 * (1 - exp(-(t-t_stage_start) / 0.2));
        
        q0 = qdes;
        tau = -damping_modifier*kp*total_weights.*(q(model.actuated_idx)-q0(model.actuated_idx)) - damping_modifier*kd*dq(model.actuated_idx) ;
end

end




%% HELPER FUNCTIONS

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
    tau = linspace(0, 1, num_steps)';
    
    % --- QUADRATIC EASE-OUT ---
    % This is much simpler than quintic.
    % It starts at max speed and decelerates to 0 velocity at the end.
    s = 1 - (1 - tau).^2;
    
    % Calculate Trajectories for all joints
    % q(t) = q0 + (qf - q0) * s(tau)
    delta_q = (qf - q0)'; % Transpose to row vector for broadcasting
    
    q = repmat(q0', num_steps, 1) + (s * delta_q);

    % figure(5);
    % plot(tau, q, 'LineWidth', 2);
    % grid on;
    % 
    % % Labeling
    % xlabel('Time (s)');
    % ylabel('Position');
    % title('Minimum Jerk Trajectory');
    
    % Dynamic Legend Generation
    % num_joints = length(q0);
    % legend_labels = cell(1, num_joints);
    % for i = 1:num_joints
    %     legend_labels{i} = sprintf('Joint %d', i);
    % end
    % legend(legend_labels, 'Location', 'best');

end

function q_sol = solveSingleFootIK(model, env, foot, p_target_ankle, p_target_toe, q0)
    % 1. Setup Frame
    q_local = q0;
    
    % Setup Indices
    act_indices = model.actuated_idx;
    x0 = q_local(act_indices); 
    
    % 2. Setup Targets and Masks
    % Initialize all targets to current pos (or zero) and mask to 0
    p1_t = zeros(3,1); p2_t = zeros(3,1); 
    p3_t = zeros(3,1); p4_t = zeros(3,1);
    
    if strcmp(env, 'body')
        q_local(1:3) = [0; 0; 1.0006];
        q_local(4:6) = [0; 0; 0]; 
        % disp(foot)
        % disp('body')
    end

    switch foot
        case 'left'
            active_mask = [1; 1; 0; 0]; % Left Ankle, Left Toe
            p1_t = p_target_ankle;
            p2_t = p_target_toe;
        case 'right'
            active_mask = [0; 0; 1; 1]; % Right Ankle, Right Toe
            p3_t = p_target_ankle;
            p4_t = p_target_toe;
    end

    % 3. Options
    options = optimoptions('fmincon', ...
        'Algorithm', 'sqp', ... 
        'Display', 'off', ...
        'MaxIterations', 500, ...
        'ConstraintTolerance', 1e-4, ...
        'StepTolerance', 1e-6);

    % 4. Objective Function
    % We pass the targets and the mask to the cost function
    fun = @(x) objectiveFunction(x, act_indices, q_local, model, ...
                                 p1_t, p2_t, p3_t, p4_t, active_mask);
    
    % 5. Run Solver (Unconstrained)
    nonlcon = [];
    [x_sol, fval, exitflag, output] = fmincon(fun, x0, [], [], [], [], [], [], nonlcon, options);
    % fprintf('  Final Cost: %e\n', fval);
    % 6. Reconstruct
    q_sol = q0;                 
    q_sol(act_indices) = x_sol; 

    if exitflag <= 0

        fprintf("Single Foot Solver didn't converge, Final Cost: %e\n", fval);
    end
end

function cost = objectiveFunction(x, indices, q_template, model, p1_tgt, p2_tgt, p3_tgt, p4_tgt, active)
    % OBJECTIVE: Minimize Error of Selected Feet + Regularization
    
    % Reconstruct q
    q_current = q_template;
    q_current(indices) = x;
    
    % Forward Kinematics Feet
    [p1_curr, p2_curr, p3_curr, p4_curr] = computeFootPositions(q_current, model);
    
    % Calculate Diff Vectors
    % LOGIC CHECK: We multiply by 'active' (not ~active).
    % If active(1) is 1, we count this error. If 0, the error becomes [0;0;0].
    foot_weighting = [1; 1; 1];
    diff1 = (p1_curr - p1_tgt) * active(1).* foot_weighting;
    diff2 = (p2_curr - p2_tgt) * active(2).* foot_weighting;
    diff3 = (p3_curr - p3_tgt) * active(3).* foot_weighting;
    diff4 = (p4_curr - p4_tgt) * active(4).* foot_weighting;


    
    % Stack errors into one large vector
    err_vec = [diff1; diff2; diff3; diff4];
    
    % Sum of Squares (Primary Cost)
    cost_tracking = sum(err_vec.^2);
    
    % Regularization (Secondary Cost)
    % Keeps joints close to initial guess to prevent flipping
    q_diff = x - q_template(indices);
    cost_reg = sum(q_diff.^2); % Small weight
    
    cost = cost_tracking + cost_reg;
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

    q_original = q0;
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
        'MaxIterations', 2000, ...
        'ConstraintTolerance', 1e-4, ... % STRICT enforcement (0.1mm)
        'StepTolerance', 1e-6);

    % 6. Objective Function (Scalar)
    % Minimizes CoM Error (weighted)
    fun = @(x) solveFootIK_objectiveFunction(x, act_indices, q0, model, p1_tgt, p2_tgt, p3_tgt, p4_tgt, active_flags);

    % 7. Constraint Function (Non-linear Equalities)
    % Enforces Foot Position = Target strictly
    % nonlcon = @(x) constraintFunction(x, act_indices, q0, model, p1_tgt, p2_tgt, p3_tgt, p4_tgt, active_flags);
    nonlcon = [[], []];

    % 8. Run the Solver
    % fmincon(fun, x0, A, b, Aeq, beq, lb, ub, nonlcon, options)
    lb = []; ub = []; 
    [x_sol, fval, exitflag, output] = fmincon(fun, x0, [], [], [], [], lb, ub, nonlcon, options);

    % 9. Reconstruct Solution
    q_sol = q_original;                 
    q_sol(act_indices) = x_sol; 

    % 10. Convergence Check
    if exitflag <= 0
        fprintf('error: %e', fval)
        warning('IK Solver did not converge perfectly.');
    end
end

function cost = solveFootIK_objectiveFunction(x, indices, q_template, model, p1_tgt, p2_tgt, p3_tgt, p4_tgt, active)
    % OBJECTIVE: Minimize Weighted CoM Error
    
    % Reconstruct q
    q_current = q_template;
    q_current(indices) = x;

    % Forward Kinematics Feet
    [p1_curr, p2_curr, p3_curr, p4_curr] = computeFootPositions(q_current, model);

    
    % For every ACTIVE foot, append its XYZ error to ceq.
    % The solver must drive these values to 0.
    % diff1 = (p1_curr - p1_tgt) * (~active(1));
    % diff2 = (p2_curr - p2_tgt) * (~active(2));
    % diff3 = (p3_curr - p3_tgt) * (~active(3));
    % diff4 = (p4_curr - p4_tgt) * (~active(4));
    foot_weighting = [1; 1; 1];
    diff1 = (p1_curr - p1_tgt).*foot_weighting;
    diff2 = (p2_curr - p2_tgt).*foot_weighting;
    diff3 = (p3_curr - p3_tgt).*foot_weighting;
    diff4 = (p4_curr - p4_tgt).*foot_weighting;
    
    % % Weighted Error Vector
    % diff_com = (r_com_curr - com_t)*10; 
    err_vec = [diff1; diff2; diff3; diff4];
    cost_tracking = sum(err_vec.^2);
    % Return Scalar Sum of Squares

    q_diff = x - q_template(indices);
    cost_reg = sum(q_diff.^2)*0.1; % Small weight
    
    cost = cost_tracking + cost_reg;

end

function [c, ceq] = solveFootIK_constraintFunction(x, indices, q_template, model, p1_t, p2_t, p3_t, p4_t, active)
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

% -----------------------------------------------------------------------
% --------------- Make it step left and right ---------------------------
% -----------------------------------------------------------------------
% function tau = studentController(t, s, model, params)
% % This function implements a Finite State Machine (FSM) for dynamic
% % push recovery (stepping).
% 
% % --- FSM State ---
% persistent robot_state;
% persistent state_change;
% persistent qdes;
% persistent traj;
% persistent traj_idx;
% persistent last_debug_bucket;
% persistent last_ik_bucket;
% persistent persist_params;
% persistent direction;
% persistent t_stage_start;
% 
% debug_interval = 0.1;
% ik_interval = 0.05;
% 
% % --- Initialize FSM on first run ---
% if isempty(robot_state)
%     disp('initialize starting state')
%     persist_params = studentParams(model);
%     robot_state = 'stage_0'; 
%     state_change = true;
%     traj_idx = 1;
%     last_debug_bucket = -1;
%     last_ik_bucket = -1;
% end
% 
% % --- State vector components ---
% q = s(1 : model.n);
% dq = s(model.n+1 : 2*model.n);
% 
% switch robot_state
%     case 'stage_0'
%         if state_change
%             state_change = false;
%         end
%         kp = 1800; kd = 300;
%         x0 = getInitialState(model);
%         q0 = x0(1:model.n);
%         tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx);
% 
%         % --- DETECTION LOGIC ---
%         a_com = [0;0;0];
%         if t > 0.01
%             [~, v_com] = computeComPosVel(q, dq, model);
%             a_com = v_com/t;
%         end
% 
%         % Threshold: 0.05 (Fast trigger)
%         if abs(a_com(1)) > 0.05
%             state_change = true;
%             robot_state = 'stage_1';
% 
%             % --- CASE 1: Pushed LEFT (+Accel) -> Step LEFT ---
%             if a_com(1) > 0
%                 direction = 'left';
%                 fprintf('Detected Push LEFT (a=%.2f). Stepping LEFT.\n', a_com(1));
% 
%                 % MIRROR of Right Step Logic:
%                 % Swing: Left (Target) | Stance: Right (Fixed)
% 
%                 % Stance (Right): Must use *1.2 multiplier like the working Left Stance
%                 persist_params.stage1 = [ ...
%                     [0.0921+persist_params.foot_dx;   0.1305+persist_params.foot_dy;     0.3], ... % L Swing Target
%                     [-0.0879+persist_params.foot_dx;  0.1305+persist_params.foot_dy;     0.3], ... 
%                     [0.0921;                         -0.1305-persist_params.foot_dy*1.2; -0.3], ... % R Stance Fixed (*1.2)
%                     [-0.0879;                        -0.1305-persist_params.foot_dy*1.2; -0.3]];    
% 
%                 % Single Foot IK Target (Left)
%                 persist_params.p_target_ankle = [0.0921+persist_params.foot_dx;  0.1305+persist_params.foot_dy; 0.3];
%                 persist_params.p_target_toe   = [-0.0879+persist_params.foot_dx; 0.1305+persist_params.foot_dy; 0.3];
% 
%                 % Mask: Fix Right (3,4), Move Left (1,2)
%                 persist_params.mask = [0;0;1;1];
% 
%                 % Gains: Left=Swing(High), Right=Stance(Low)
%                 persist_params.stage1_kp_left = 1700;  
%                 persist_params.stage1_kd_left = 300;
%                 persist_params.stage1_kp_right = 1400; 
%                 persist_params.stage1_kd_right = 300;
% 
%             % --- CASE 2: Pushed RIGHT (-Accel) -> Step RIGHT (Original Working) ---
%             else
%                 direction = 'right';
%                 fprintf('Detected Push RIGHT (a=%.2f). Stepping RIGHT.\n', a_com(1));
% 
%                 % Swing: Right (Target) | Stance: Left (Fixed)
% 
%                 % Stance (Left): Uses *1.2 multiplier
%                 persist_params.stage1 = [ ...
%                     [0.0921;                          0.1305+persist_params.foot_dy*1.2; -0.3], ... % L Stance Fixed (*1.2)
%                     [-0.0879;                         0.1305+persist_params.foot_dy*1.2; -0.3], ... 
%                     [0.0921+persist_params.foot_dx;  -0.1305-persist_params.foot_dy;      0.3], ... % R Swing Target
%                     [-0.0879+persist_params.foot_dx; -0.1305-persist_params.foot_dy;      0.3]];    
% 
%                 % Single Foot IK Target (Right)
%                 persist_params.p_target_ankle = [0.0921+persist_params.foot_dx; -0.1305-persist_params.foot_dy; 0.3];
%                 persist_params.p_target_toe   = [-0.0879+persist_params.foot_dx; -0.1305-persist_params.foot_dy; 0.3];
% 
%                 % Mask: Fix Left (1,2), Move Right (3,4)
%                 persist_params.mask = [1;1;0;0];
% 
%                 % Gains: Left=Stance(Low), Right=Swing(High)
%                 persist_params.stage1_kp_left = 1400; 
%                 persist_params.stage1_kd_left = 300;
%                 persist_params.stage1_kp_right = 1700;
%                 persist_params.stage1_kd_right = 300;
%             end
%         end
% 
%     case 'stage_1'
%         if state_change
%             state_change = false;
%             disp("Entering Stage 1: Swing")
%             traj_idx = 1;
%         end
% 
%         current_bucket = floor(t / ik_interval);
% 
%         % --- IK UPDATE ---
%         if current_bucket > last_ik_bucket
%             foot_des = persist_params.stage1;
%             q1 = q;
% 
%             % 1. Full Body IK (Enforces Stance Leg Constraints)
%             % Reverted to using 'q' (current pose) for regularization, matching working code
%             q1 = solveFootIK(model, foot_des(:,1), foot_des(:,2), foot_des(:,3), foot_des(:,4), persist_params.mask, q);
% 
%             % 2. Swing Leg IK
%             q2 = solveSingleFootIK(model, 'body', direction, persist_params.p_target_ankle, persist_params.p_target_toe, q);
% 
%             % 3. Splicing
%             if strcmp(direction, 'left')
%                 % Left Indices: [HipRoll, HipYaw, HipPitch, Knee, Toe]
%                 idx_splice = [7, 9, 11, 13, 19];
%             else
%                 % Right Indices: [HipRoll, HipYaw, HipPitch, Knee, Toe]
%                 idx_splice = [8, 10, 12, 14, 20];
%             end
%             q1(idx_splice) = q2(idx_splice);
% 
%             % --- CRITICAL FIX: TARGET q1 IMMEDIATELY ---
%             % Previously: traj = generate_trajectory(q, q1, 2) creates [Current; Target]
%             % This caused Error=0 for the first tick of every cycle (Loss of Stiffness)
%             % New: We repeat q1 so the target is ALWAYS the IK solution.
%             traj = repmat(q1(model.actuated_idx)', 2, 1);
%             traj_idx = 1;
%             last_ik_bucket = current_bucket;
% 
%             if current_bucket > last_debug_bucket
%                  fprintf('[DEBUG t=%.2f] Re-planning. Direction: %s\n', t, direction);
%             end
%         end
% 
%         % --- CONTROLLER ---
%         kp_left = persist_params.stage1_kp_left;
%         kd_left = persist_params.stage1_kd_left;
%         kp_right = persist_params.stage1_kp_right;
%         kd_right = persist_params.stage1_kd_right;
% 
%         % Weights: [Roll, Yaw, Pitch, Knee, Toe]
%         % Exact weights from working code [1, 1, 0.1, 0.2, 0.1]
%         weights = [1, 1, 0.1, 0.2, 0.1];
% 
%         % Interleaved Gain Array Construction
%         C = [weights(:), weights(:)].'; % 2x5 matrix
%         total_weights = C(:);           % 10x1 Interleaved [L; R; L; R...]
% 
%         kp_raw = [kp_left; kp_right; kp_left; kp_right; kp_left; kp_right; kp_left; kp_right; kp_left; kp_right];
%         kd_raw = [kd_left; kd_right; kd_left; kd_right; kd_left; kd_right; kd_left; kd_right; kd_left; kd_right];
% 
%         kp_arr = total_weights .* kp_raw;
%         kd_arr = kd_raw; 
% 
%         q_des_traj = traj(traj_idx,:).';
%         tau = -kp_arr.*(q(model.actuated_idx)-q_des_traj) - kd_arr.*dq(model.actuated_idx);
% 
%         % --- TOUCHDOWN DETECTION ---
%         [p1, p2, p3, p4] = computeFootPositions(q, model);
%         p_feet = [p1, p2, p3, p4];
% 
%         % Filter based on direction (Matching logic from working code)
%         if strcmp(direction, 'left')
%             % Looking at Left Foot (Cols 1-2)
%             p_check = p_feet(:, 1:2);
%         else
%             % Looking at Right Foot (Cols 3-4)
%             p_check = p_feet(:, 3:4);
%         end
% 
%         % Check Z height of the specific foot
%         if t > 0.5 && min(p_check(3,:)) < 1e-3
%             disp('Touchdown detected -> Stage End')
%             robot_state = 'stage_end';
%             state_change = true;
%         end
% 
%         % Advance Trajectory
%         error_dist = norm((q(model.actuated_idx)-q_des_traj))^2;
%         if error_dist < persist_params.tolerance
%              traj_idx = min(traj_idx + 1, height(traj));
%         end
% 
%         if current_bucket > last_debug_bucket
%             fprintf('[DEBUG t=%.2f] Error: %.4f | Idx: %i\n', t, error_dist, traj_idx);
%             last_debug_bucket = current_bucket;
%         end
% 
%     case 'stage_end'
%         if state_change
%             state_change = false;
%             qdes = q; % Lock Pose
%             t_stage_start = t;
%             disp("Stage End: Locking Position")
%         end
% 
%         kp = 1800; kd = 300;
% 
%         % Stage End Weights
%         w = [1, 1, 0.7, 0.6, 0.4];
%         C = [w(:), w(:)].';
%         total_weights = C(:);
% 
%         damping_modifier = 0.3 + 0.7 * (1 - exp(-(t-t_stage_start) / 0.2));
% 
%         tau = -damping_modifier*kp*total_weights.*(q(model.actuated_idx)-qdes(model.actuated_idx)) - damping_modifier*kd*dq(model.actuated_idx);
% end
% end
% 
% %% HELPER FUNCTIONS
% function [q, t] = generate_trajectory(q_start, q_des, num_steps)
%     q0 = q_start(:);
%     qf = q_des(:);
%     tau = linspace(0, 1, num_steps)';
%     s = 1 - (1 - tau).^2;
%     delta_q = (qf - q0)'; 
%     q = repmat(q0', num_steps, 1) + (s * delta_q);
% end
% 
% function q_sol = solveSingleFootIK(model, env, foot, p_target_ankle, p_target_toe, q0)
%     q_local = q0;
%     act_indices = model.actuated_idx;
%     x0 = q_local(act_indices); 
% 
%     p1_t = zeros(3,1); p2_t = zeros(3,1); 
%     p3_t = zeros(3,1); p4_t = zeros(3,1);
% 
%     if strcmp(env, 'body')
%         q_local(1:3) = [0; 0; 1.0006];
%         q_local(4:6) = [0; 0; 0]; 
%     end
%     switch foot
%         case 'left'
%             active_mask = [1; 1; 0; 0]; 
%             p1_t = p_target_ankle;
%             p2_t = p_target_toe;
%         case 'right'
%             active_mask = [0; 0; 1; 1]; 
%             p3_t = p_target_ankle;
%             p4_t = p_target_toe;
%     end
%     options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off', ...
%         'MaxIterations', 500, 'ConstraintTolerance', 1e-4, 'StepTolerance', 1e-6);
%     fun = @(x) objectiveFunction(x, act_indices, q_local, model, ...
%                                  p1_t, p2_t, p3_t, p4_t, active_mask);
% 
%     nonlcon = [];
%     [x_sol, fval, exitflag, ~] = fmincon(fun, x0, [], [], [], [], [], [], nonlcon, options);
%     q_sol = q0;                 
%     q_sol(act_indices) = x_sol; 
%     if exitflag <= 0
%         fprintf("Single Foot Solver didn't converge, Final Cost: %e\n", fval);
%     end
% end
% 
% function cost = objectiveFunction(x, indices, q_template, model, p1_tgt, p2_tgt, p3_tgt, p4_tgt, active)
%     q_current = q_template;
%     q_current(indices) = x;
%     [p1_curr, p2_curr, p3_curr, p4_curr] = computeFootPositions(q_current, model);
% 
%     foot_weighting = [1; 1; 1];
%     % Included the 'active' mask fix. This is safe and robust.
%     diff1 = (p1_curr - p1_tgt) * active(1).* foot_weighting;
%     diff2 = (p2_curr - p2_tgt) * active(2).* foot_weighting;
%     diff3 = (p3_curr - p3_tgt) * active(3).* foot_weighting;
%     diff4 = (p4_curr - p4_tgt) * active(4).* foot_weighting;
% 
%     err_vec = [diff1; diff2; diff3; diff4];
%     cost_tracking = sum(err_vec.^2);
%     q_diff = x - q_template(indices);
%     cost_reg = sum(q_diff.^2); 
%     cost = cost_tracking + cost_reg;
% end
% 
% function q_sol = solveFootIK(model, p1_tgt, p2_tgt, p3_tgt, p4_tgt, feet_active, q0)
%     q_original = q0;
%     min_z = min([p1_tgt(3), p2_tgt(3), p3_tgt(3), p4_tgt(3)]);
%     if min_z < 0
%         p1_tgt(3) = p1_tgt(3) - min_z;
%         p2_tgt(3) = p2_tgt(3) - min_z;
%         p3_tgt(3) = p3_tgt(3) - min_z;
%         p4_tgt(3) = p4_tgt(3) - min_z;
%         q0(3) = q0(3) - min_z;
%     end
%     act_indices = model.actuated_idx;
%     x0 = q0(act_indices); 
%     active_flags = feet_active(:);
% 
%     options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off', ...
%         'MaxIterations', 2000, 'ConstraintTolerance', 1e-4, 'StepTolerance', 1e-6);
% 
%     fun = @(x) solveFootIK_objectiveFunction(x, act_indices, q0, model, p1_tgt, p2_tgt, p3_tgt, p4_tgt, active_flags);
%     nonlcon = [[], []];
%     [x_sol, fval, exitflag, ~] = fmincon(fun, x0, [], [], [], [], [], [], nonlcon, options);
% 
%     q_sol = q_original;                 
%     q_sol(act_indices) = x_sol; 
%     if exitflag <= 0
%         fprintf('error: %e', fval)
%         warning('IK Solver did not converge perfectly.');
%     end
% end
% 
% function cost = solveFootIK_objectiveFunction(x, indices, q_template, model, p1_tgt, p2_tgt, p3_tgt, p4_tgt, active)
%     q_current = q_template;
%     q_current(indices) = x;
%     [p1_curr, p2_curr, p3_curr, p4_curr] = computeFootPositions(q_current, model);
% 
%     foot_weighting = [1; 1; 1];
% 
%     % --- CRITICAL FIX: Multiply by 'active' mask ---
%     % Ensures Stance IK doesn't try to fit Swing targets
%     diff1 = (p1_curr - p1_tgt) * active(1) .* foot_weighting;
%     diff2 = (p2_curr - p2_tgt) * active(2) .* foot_weighting;
%     diff3 = (p3_curr - p3_tgt) * active(3) .* foot_weighting;
%     diff4 = (p4_curr - p4_tgt) * active(4) .* foot_weighting;
% 
%     err_vec = [diff1; diff2; diff3; diff4];
%     cost_tracking = sum(err_vec.^2);
%     q_diff = x - q_template(indices);
%     cost_reg = sum(q_diff.^2)*0.1; 
%     cost = cost_tracking + cost_reg;
% end
% 
% function [c, ceq] = solveFootIK_constraintFunction(x, indices, q_template, model, p1_t, p2_t, p3_t, p4_t, active)
%     q_current = q_template;
%     q_current(indices) = x;
%     [p1, p2, p3, p4] = computeFootPositions(q_current, model);
%     c = [];
%     ceq = [];
%     if active(1), ceq = [ceq; p1 - p1_t]; end
%     if active(2), ceq = [ceq; p2 - p2_t]; end
%     if active(3), ceq = [ceq; p3 - p3_t]; end
%     if active(4), ceq = [ceq; p4 - p4_t]; end
% end