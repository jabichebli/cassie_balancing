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

% print(params)

debug_interval = 0.05;

% --- Initialize FSM on first run ---
if isempty(robot_state)
    disp(t)
    disp('initialize starting state')
    robot_state = 'stage_1'; % Start by standing on two feet
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
    case 'stage_1'
        if state_change
            state_change = false;
            foot_des = params.stage1;
            qdes = solveFootIK(model, foot_des(:,1), foot_des(:,2), foot_des(:,3), foot_des(:,4), [0;0;1;1], q);
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
        kp_arr = ones([5,2]);
        kp_arr(1:4,1) = kp_arr(1:4,1) .* 500;
        kp_arr(5,1) = kp_arr(5,1) .* 500;

        % right leg
        kp_arr(:,2) = kp_arr(:,2) * 1800;
        kp_one = [kp_arr(:,1) ; kp_arr(:,2)];

        % kp = 1800 ; 
        kd = 100 ;
        q1 = traj(traj_idx,:).';
        % q1 = qdes(model.actuated_idx);
        % tau = -kp_one.*(q(model.actuated_idx)-q1(model.actuated_idx)) - kd*dq(model.actuated_idx) ;
        tau = -kp_one.*(q(model.actuated_idx)-q1) - kd*dq(model.actuated_idx) ;

        error_dist = norm((q(model.actuated_idx)-q1))^2;
    
        current_bucket = floor(t / debug_interval);
    
        % If we have moved to a new bucket, print and update
        if current_bucket > last_debug_bucket
            fprintf('[DEBUG t=%.2f] Error Distance: %d | Traj Idx: %i\n', t, error_dist, traj_idx);
            last_debug_bucket = current_bucket;
        end

        % Check if within tolerance
        if error_dist < params.tolerance
            fprintf("Reached tolerance for traj idx %i at %d\n", traj_idx, t)
            traj_idx = traj_idx + 1;
            if traj_idx > height(traj)
                robot_state = 'stage_2';
                state_change = true;
                disp(q)
                % traj_idx = 1;
            end
        end
        
        % if traj_idx == length(traj)
        %     robot_state = 'stage_end';
        %     state_change = true;
        %     disp(q)
        %     traj_idx = 1;
        % end


        

    case 'stage_2'
        if state_change
            state_change = false;
            foot_des = params.stage2;
            qdes = solveFootIK(model, foot_des(:,1), foot_des(:,2), foot_des(:,3), foot_des(:,4), [0;0;1;1], q);
            disp("changing to stage 2")
            % disp('differences')
            traj = generate_trajectory(q(model.actuated_idx), qdes(model.actuated_idx), 2);
            traj_idx = 1;
            % % disp(traj)
            % disp(norm(q - qdes))
            % disp('qdes')
            % disp(qdes)
        end
        %% Base controller to keep stance for now
        kp_left = 300;
        kd_left = 30;

        kp_right = 1300;
        kd_right = 300;
        
        % left leg
        kp_arr = [kp_left; kp_right; kp_left; kp_right; kp_left; kp_right; kp_left; kp_right; kp_left*0.5; kp_right*0.5];

        kd_arr = [kd_left; kd_right; kd_left; kd_right; kd_left; kd_right; kd_left; kd_right; kd_left*0.5; kd_right*0.5];

        % kp_arr = ones([5,2]);
        % kp_arr(1:4,1) = kp_arr(1:4,1) .* 100;
        % kp_arr(5,1) = kp_arr(5,1) .* 10;
        % 
        % % right leg
        % kp_arr(:,2) = kp_arr(:,2) .* 1800;
        % kp_one = [kp_arr(:,1) ; kp_arr(:,2)];
        % 
        % % kp = 1800 ; 
        % kd = 5 ;
        % kd_arr = ones([10,1]);
        % kd_arr(6:10) = kd_arr(6:10).*300;
        % kd_arr()
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

        [p1_curr, p2_curr, p3_curr, p4_curr] = computeFootPositions(q, model);
        p_foot = [p1_curr, p2_curr, p3_curr, p4_curr];
        p_foot = p_foot(:,1:2);

        if min(p_foot(3,:)) < 1e-3
            disp('touchdown')
            robot_state = 'stage_end';
            state_change = true;
            disp(q)
        end

        % Check if within tolerance
        if error_dist < params.tolerance
            fprintf("Reached tolerance for traj idx %i at %d\n", traj_idx, t)
            traj_idx = traj_idx + 1;



            % if traj_idx > height(traj)
            %     robot_state = 'stage_end';
            %     state_change = true;
            %     disp(q)
            %     % traj_idx = 1;
            % end
        end

    case 'stage_end'
        if state_change
            state_change = false;
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
