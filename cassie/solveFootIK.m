function q_sol = solveFootIK(model, p1_tgt, p2_tgt, p3_tgt, p4_tgt, q0)
% SOLVEFOOTIK Solves for joint configuration q given target foot positions.
%
%   q_sol = solveFootIK(model, p1_tgt, p2_tgt, p3_tgt, p4_tgt, q0)
%
%   INPUTS:
%       model   - Robot model structure. MUST contain 'actuated_idx'.
%       pX_tgt  - Target position vectors (3x1) for each foot
%       q0      - Initial guess for q (Nx1 vector).
%
%   OUTPUT:
%       q_sol   - The full joint vector with optimized actuated joints.

    % 1. Check input arguments
    if nargin < 6
        error('Please provide an initial guess q0 (e.g., the robot''s current state).');
    end

    % Verify that actuated_idx exists in the model
    if ~isfield(model, 'actuated_idx')
        error('The model structure must contain "actuated_idx" to identify controllable joints.');
    end

    % 2. Prepare Optimization Variables
    % We only want to solve for the actuated joints, keeping others fixed.
    act_indices = model.actuated_idx;
    x0 = q0(act_indices); % Initial guess for ONLY the actuated joints

    % 3. Define Solver Options
    options = optimoptions('lsqnonlin', ...
        'Algorithm', 'levenberg-marquardt', ... 
        'Display', 'off', ...                    
        'FunctionTolerance', 1e-6, ...           
        'StepTolerance', 1e-6);

    % 4. Define Bounds (Optional)
    % If you switch to 'trust-region-reflective', you can set bounds here
    % based on physical limits of the actuated joints.
    lb = [];
    ub = [];

    % 5. Create Objective Function
    % We pass q0 (as a template) and the indices to reconstruct the full q inside
    objectiveFunc = @(x) costFunction(x, act_indices, q0, model, p1_tgt, p2_tgt, p3_tgt, p4_tgt);

    % 6. Run the Solver
    % x_sol will contain only the values for the actuated joints
    [x_sol, resnorm, ~, exitflag] = lsqnonlin(objectiveFunc, x0, lb, ub, options);

    % --- NEW: Print the final cost ---
    fprintf('IK Solver Final Cost (Squared Error): %e\n', resnorm);
    
    % 7. Reconstruct Full Solution
    q_sol = q0;                 % Start with the initial/fixed values
    q_sol(act_indices) = x_sol; % Update the actuated joints with the solution

    % 8. Check for Convergence
    if exitflag <= 0
        warning('IK Solver did not converge perfectly. Result might be inaccurate.');
    end
end

function error_vector = costFunction(x, indices, q_template, model, p1_t, p2_t, p3_t, p4_t)
    % COSTFUNCTION Helper to reconstruct full q and calculate error.
    
    % A. Reconstruct the full q vector
    q_current = q_template;
    q_current(indices) = x;
    
    % B. Run Forward Kinematics
    [p1_curr, p2_curr, p3_curr, p4_curr] = computeFootPositions(q_current, model);
    
    % C. Calculate Error
    diff1 = p1_curr - p1_t;
    diff2 = p2_curr - p2_t;
    diff3 = p3_curr - p3_t;
    diff4 = p4_curr - p4_t;
    
    % D. Stack errors
    error_vector = [diff1; diff2; diff3; diff4];
end