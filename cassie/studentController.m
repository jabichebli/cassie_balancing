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

% print(params)

debug_interval = 0.1;
ik_interval = 0.1;


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

p_target_ankle = [-0.2052;-0.6278;0.1047+0.05];
p_target_toe = [-0.3850;-0.6244;0.1106+0.05];

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
                % persist_params.kp_left = 1300;
                % persist_params.kd_left = 300;
                % persist_params.kp_right = 300;
                % persist_params.kd_right = 30;

                persist_params.stage1_kp_right = 500;
                persist_params.stage1_kd_right = 50;
                persist_params.stage1_kp_left = 1000;
                persist_params.stage1_kd_left = 200;

                persist_params.stage2_kp_right = 300;
                persist_params.stage2_kd_right = 30;
                persist_params.stage2_kp_left = 1300;
                persist_params.stage2_kd_left = 300;

                kp_left = 500;
                kd_left = 50;
                kp_right = 1500;
                kd_right = 300;

                persist_params.kp_swing = [kp_left; kp_right;
                                           kp_left; kp_right;
                                           kp_left; kp_right;
                                           kp_left; kp_right;
                                           kp_left; kp_right];
                persist_params.kp_swing = [kd_left; kd_right;
                                           kd_left; kd_right;
                                           kd_left; kd_right;
                                           kd_left; kd_right;
                                           kd_left; kd_right];
            end
        end

    case 'stage_1'
        if state_change
            state_change = false;
            % foot_des = persist_params.stage1;
            % qdes = solveFootIK(model, foot_des(:,1), foot_des(:,2), foot_des(:,3), foot_des(:,4), persist_params.mask, q);
            % q1 = q;
            % 
            % last_q1 = q1;
            % 
            % q1 = solveSingleFootIK(model, 'world', 'left', foot_des(:,1), foot_des(:,2), q1);
            % q1 = solveSingleFootIK(model, 'body', 'right', p_target_ankle, p_target_toe, q1);
            % 
            % disp(q1 - last_q1);
            % last_q1 = q1;
            % 
            % disp("changing to stage 1")
            % traj = generate_trajectory(q(model.actuated_idx), q1(model.actuated_idx), 2);
            % traj_idx = 1;
        end

        current_bucket = floor(t / ik_interval);
        if current_bucket > last_ik_bucket
            foot_des = persist_params.stage1;
            q1 = q;
            last_q1 = q1;

            % q1 = solveFootIK(model, foot_des(:,1), foot_des(:,2), foot_des(:,3), foot_des(:,4), [1;1;0;0], q);

            q1 = solveSingleFootIK(model, 'world', 'left', foot_des(:,1), foot_des(:,2), q1);
            q1 = solveSingleFootIK(model, 'body', 'right', p_target_ankle, p_target_toe, q1);

            disp(q1 - last_q1);

            fprintf('[DEBUG Current Bucket=%i]\n', current_bucket);

            traj = generate_trajectory(q(model.actuated_idx), q1(model.actuated_idx), 2);
            traj_idx = 1;

            last_ik_bucket = current_bucket;
        end

        %% Base controller to keep stance for now
        % left leg
        kp_left = persist_params.stage1_kp_left;
        kd_left = persist_params.stage1_kd_left;
        kp_right = persist_params.stage1_kp_right;
        kd_right = persist_params.stage1_kd_right;
        % left leg
        kp_arr = [kp_left; kp_right; kp_left; kp_right; kp_left; kp_right; kp_left; kp_right; kp_left; kp_right];
        kd_arr = [kd_left; kd_right; kd_left; kd_right; kd_left; kd_right; kd_left; kd_right; kd_left; kd_right];

        % left leg
        % kp_arr = ones([5,2]);
        % kp_arr(1:4,1) = kp_arr(1:4,1) .* 500;
        % kp_arr(5,1) = kp_arr(5,1) .* 500;
        % kp_arr(:,2) = kp_arr(:,2) * 1800;
        % kp_arr = [kp_arr(:,1) ; kp_arr(:,2)];
        % 
        % kd_arr = ones([10,1]) .* 100;

        q1 = traj(traj_idx,:).';
        % q1 = qdes(model.actuated_idx);
        % tau = -kp_one.*(q(model.actuated_idx)-q1(model.actuated_idx)) - kd*dq(model.actuated_idx) ;
        tau = -kp_arr.*(q(model.actuated_idx)-q1) - kd_arr.*dq(model.actuated_idx) ;

        error_dist = norm((q(model.actuated_idx)-q1))^2;
    

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
                robot_state = 'stage_end';
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
        kp_left = persist_params.stage2_kp_left;
        kd_left = persist_params.stage2_kd_left;
        kp_right = persist_params.stage2_kp_right;
        kd_right = persist_params.stage2_kd_right;
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
            fprintf("Stage %i: Reached tolerance for traj idx %i at %d\n", traj_idx, t)
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
