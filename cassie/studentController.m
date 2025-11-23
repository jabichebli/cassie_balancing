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
persistent debug_q_act;

persistent hip_pitch;

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

    hip_pitch = [];

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
                persist_params.stage1 = [[0.0921;0.1305+persist_params.foot_dy*1.3;0],...
                                         [-0.0879;0.1305+persist_params.foot_dy*1.3;0],...
                                         [0.0921+persist_params.foot_dx;-0.1305-persist_params.foot_dy;0.15],...
                                         [-0.0879+persist_params.foot_dx;-0.1305-persist_params.foot_dy;0.15]];
                % persist_params.p_target_ankle = [0.0921+persist_params.foot_dx;-0.1305-persist_params.foot_dy;0.3];
                % persist_params.p_target_toe = [-0.0879+persist_params.foot_dx;-0.1305-persist_params.foot_dy;0.3];
                
                persist_params.stage2 = [[0.0921-persist_params.foot_dx*2;0.1305;0],...
                                        [-0.0879-persist_params.foot_dx*2;0.1305;0],...
                                        [0.0921+persist_params.foot_dx*2;-0.1305-persist_params.foot_dy*3;0],...
                                        [-0.0879+persist_params.foot_dx*2;-0.1305-persist_params.foot_dy*3;0]];
                persist_params.mask = [1;1;0;0];
                % persist_params.kp_left = 1300;
                % persist_params.kd_left = 300;
                % persist_params.kp_right = 300;
                % persist_params.kd_right = 30;

                persist_params.stage1_kp_left = 1500;
                persist_params.stage1_kd_left = 300;
                persist_params.stage1_kp_right = 1000;
                persist_params.stage1_kd_right = 200;


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

            foot_des = persist_params.stage1;
            q_tmp = solveSingleFootIK(model, 'body', 'right', foot_des(:,3), foot_des(:,4), q);
            hip_abductor = q_tmp(8);
            traj_idx = 1;
        end
        current_bucket = floor(t / ik_interval);



        if current_bucket > last_ik_bucket
            foot_des = persist_params.stage1;
            q1 = solveFootIK(model, foot_des(:,1), foot_des(:,2), foot_des(:,3), foot_des(:,4), [1;1;0;0], q);
            q2 = solveSingleFootIK(model, 'body', 'right', foot_des(:,3), foot_des(:,4), q);
            q1([8, 10, 12, 14, 20]) = q2([8, 10, 12, 14, 20]);
            % q1(7) = -hip_abductor * (1 - exp(-t / 0.1));
            % q1(9) = q1(9) - 0.3 * (1 - exp(-t / 0.13));
            % q1(8) = q1(8) + deg2rad(15 * (1 - exp(-t / 0.5)))/2;
            fprintf('[DEBUG Current Bucket=%i]\n', current_bucket);

            traj = generate_trajectory(q(model.actuated_idx), q1(model.actuated_idx), 2);
            % traj_idx = 1;

            last_ik_bucket = current_bucket;
        end

        q1 = traj(traj_idx,:).';

        hip_pitch = [hip_pitch, [t; q1(5); q(5)]];
        debug_q_act = [debug_q_act, [t; q1; q(model.actuated_idx)]];

        %% Base controller to keep stance for now

        kp_left = persist_params.stage1_kp_left;
        kd_left = persist_params.stage1_kd_left;
        kp_right = persist_params.stage1_kp_right;
        kd_right = persist_params.stage1_kd_right;

        % weights
        % hip abduction, hip rotation, hip flexion, knee joint, toe joint
        % left_weights = [1, 1, 0.09, 0.4, 0.8];
        left_weights = [1, 1, 1, 1, 0.8];
        right_weights = [1, 1, 1, 1, 1];

        C = [left_weights(:),right_weights(:)].';
        total_weights = C(:);

        kp_arr = total_weights.*[kp_left; kp_right; kp_left; kp_right; kp_left; kp_right; kp_left; kp_right; kp_left; kp_right];
        kd_arr = total_weights.*[kd_left; kd_right; kd_left; kd_right; kd_left; kd_right; kd_left; kd_right; kd_left; kd_right];

        
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
            % disp(q)
        end

        % Check if within tolerance
        if error_dist < persist_params.tolerance
            fprintf("Reached tolerance for traj idx %i at %.2f\n", traj_idx, t)
            % [p1_curr, p2_curr, p3_curr, p4_curr] = computeFootPositions(q, model)
            traj_idx = traj_idx + 1;
            if traj_idx > height(traj)
                robot_state = 'stage_end';
                state_change = true;
                disp('Finished Trajectory for Stage 1')
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
            fprintf('Reached Stage End at Time: %.2d. Holding Position', t)
            
            figure; hold on;
            plot(hip_pitch(1,:), hip_pitch(2,:), 'DisplayName', 'desired');
            plot(hip_pitch(1,:), hip_pitch(3,:), 'DisplayName', 'actual');
            title("Hip pitch over time")
            legend on;

            figure; hold on;
            % plot(debug_q_act(1,:), debug_q_act(2:end,:));
            title("Desired Joints over time")
            subplot(2,1,1) ;
            plot(debug_q_act(1,:), debug_q_act(2:2:10,:)) ;
            grid on ; title('Left Desired Angles') ; legend('abduction','rotation','flexion','knee','toe') ;
            ylim([-2, 1])

            subplot(2,1,2) ;
            plot(debug_q_act(1,:), debug_q_act(12:2:end,:)) ;
            grid on ; title('Left Actual Angles') ; legend('abduction','rotation','flexion','knee','toe') ;
            ylim([-2, 1])


            figure; hold on;
            % plot(debug_q_act(1,:), debug_q_act(2:end,:));
            title("Desired Joints over time")
            subplot(2,1,1) ;
            plot(debug_q_act(1,:), debug_q_act(3:2:11,:)) ;
            grid on ; title('Right Desired Angles') ; legend('abduction','rotation','flexion','knee','toe') ;
            ylim([-2, 1])

            subplot(2,1,2) ;
            plot(debug_q_act(1,:), debug_q_act(13:2:end,:)) ;
            grid on ; title('Right Actual Angles') ; legend('abduction','rotation','flexion','knee','toe') ;
            ylim([-2, 1])
    

            clear functions;
            qdes = q;
            t_stage_start = t;
            state_change = false;
            
        end
        
        kp = 1800 ;
        kd = 300 ;

        % left_weights = [1, 1, 0.7, 0.6, 0.4];
        left_weights = [1, 1, 1, 1, 1];
        right_weights = left_weights;
        % right_weights = [1, 1, 0.2, 0.2, 0.1];

        C = [left_weights(:),right_weights(:)].';   %'
        total_weights = C(:);
        
        % damping_modifier = 0.3 + 0.7 * (1 - exp(-(t-t_stage_start) / 0.2));
        damping_modifier = 1;

        
        q0 = qdes;
        tau = -damping_modifier*kp*total_weights.*(q(model.actuated_idx)-q0(model.actuated_idx)) - damping_modifier*kd*dq(model.actuated_idx) ;
end

end




%% HELPER FUNCTIONS
