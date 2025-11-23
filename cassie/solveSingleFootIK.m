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
    fprintf('  Final Cost: %e\n', fval);
    % 6. Reconstruct
    q_sol = q0;                 
    q_sol(act_indices) = x_sol; 

    if exitflag <= 0
        fprintf('  Final Cost: %e\n', fval);
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
    diff1 = (p1_curr - p1_tgt) * active(1);
    diff2 = (p2_curr - p2_tgt) * active(2);
    diff3 = (p3_curr - p3_tgt) * active(3);
    diff4 = (p4_curr - p4_tgt) * active(4);
    
    % Stack errors into one large vector
    err_vec = [diff1; diff2; diff3; diff4];
    
    % Sum of Squares (Primary Cost)
    cost_tracking = sum(err_vec.^2);
    
    % Regularization (Secondary Cost)
    % Keeps joints close to initial guess to prevent flipping
    q_diff = x - q_template(indices);
    cost_reg = sum(q_diff.^2) .* 0.1; % Small weight
    
    cost = cost_tracking + cost_reg;
end