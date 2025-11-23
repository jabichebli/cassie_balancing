

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
    fun = @(x) objectiveFunction(x, act_indices, q0, model, p1_tgt, p2_tgt, p3_tgt, p4_tgt, active_flags);

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

function cost = objectiveFunction(x, indices, q_template, model, p1_tgt, p2_tgt, p3_tgt, p4_tgt, active)
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