function test_cassie_pose()
close all ;

% add paths
startup;

% Load Cassie model and set Initial configuration
model = load('cassie_model.mat') ;

% Initial configuration
[x0, model] = getInitialState(model.model);


% Get STUDENT Control Parameters
params = studentParams(model);

q = x0(1 : model.n);
dq = x0(model.n+1 : 2*model.n);

[r_com, v_com] = computeComPosVel(q, dq, model);
[p1, p2, p3, p4] = computeFootPositions(q, model);

% disp(compute_COM_pos);

r_com_xy = r_com(1:2);
v_com_xy = v_com(1:2);

poly_points_x = [p1(1), p2(1), p3(1), p4(1)];
poly_points_y = [p1(2), p2(2), p3(2), p4(2)];

disp('poly points')
disp(poly_points_x)
disp(poly_points_y)
disp(r_com_xy)
disp(v_com_xy)

% ankle_pos_des_1 = [0, 0.1305, 0.1];

% foot_des = [[0.0921+0.0900;0.1305;0.1], [-0.0879+0.0900;0.1305;0.1], [0.0921;-0.1305;0], [-0.0879;-0.1305;0]];
foot_des = [[0.0921;0.1305;0.1], [-0.0879;0.1305;0.1], [0.0921;-0.1305;0], [-0.0879;-0.1305;0]];
% foot_des = [[0.0921;0.1305;0], [-0.0879;0.1305;0], [0.0921;-0.1305;0], [-0.0879;-0.1305;0]];

q1 = solveFootIK(model, foot_des(:,1), foot_des(:,2), foot_des(:,3), foot_des(:,4), [1;1;0;0], [0;0;-0.05], q);

% disp(q)
% disp(q1)

disp('com delta')

disp(r_com)
disp(v_com)

[r_com1, v_com1] = computeComPosVel(q1, dq, model);
disp(r_com1)
disp(v_com1)

disp(r_com1 - r_com)
% params.stage2 = [[0.0921;0.505;0], [-0.0879;0.505;0], [0.0921;-0.1305;0], [-0.0879;-0.1305;0]];
% 
% params.q1 = solveFootIK(model, [0.0921;0.1305;0.1], [-0.0879;0.1305;0.1], [0.0921;-0.1305;0], [-0.0879;-0.1305;0], q);
% params.t1 = 0.5;
% 
% params.q2 = solveFootIK(model, [0.0921;0.405;0], [-0.0879;0.405;0], [0.0921;-0.1305;0], [-0.0879;-0.1305;0], q);
% params.t2 = 0.4;

% q_diff = params.q1 - q;
% print(q_diff)

% % ODE options
% time_inter = [0 5] ;
% odeopts = odeset('Events', @falldetect);
% externalForce_fun = @ExternalForce ;
% 
% %% Simulation 
% disp('Simulating...') ;
% tic
% 
% [t_vec, x_vec] = ode15s( @cassie_eom, time_inter, x0, odeopts, model, params, externalForce_fun) ;
% 
% toc
% disp(['Simulated for ' num2str(t_vec(end)), 's'])
% 
% %% Calculate Score
% score = calcScore(t_vec', x_vec', model);
% disp(['Score: ', num2str(score)])
% 
% %%
% r_com = zeros(length(t_vec), 3) ;
% for i = 1 : size(x_vec,1)
%     r_com(i,:) = compute_COM_pos(model, x_vec(i,1:model.NB))' ;
% end
% 
% %% Calculate control signals
% disp('Computing control signals...') ;
% xdot_vec = zeros(size(x_vec)) ;
% tau_vec = zeros(length(t_vec), 20) ;
% for j=1:length(t_vec)
%     [xdot_,tau_] = cassie_eom(t_vec(j),x_vec(j,:)', model,params,externalForce_fun) ;
%     xdot_vec(j,:) = xdot_' ;
%     tau_vec(j,:) = tau_' ;
% end
% 
% %% Plots and animation
% 
% disp('Graphing...') ;
% % Plot COM position, base orientation, joint angles
% figure() ; 
%     subplot(3,1,1);plot(t_vec, r_com) ;grid ; title('com positions x-y-z') ;hold; legend('x','y','z') ;
%     subplot(3,1,2); plot(t_vec, x_vec(:,4:6)) ; grid ; title('base angles') ; 
%     subplot(3,1,3); plot(t_vec, x_vec(:,7:model.n)) ; grid ; title('joint angles') ; 
% 
% % Plot Base (Pelvis) Position
% figure ; plot(t_vec, x_vec(:,1:3)) ; grid on ;
%     title('Base (Pelvis) Translation') ; legend('x','y','z') ;
% 
% % Plot Base (Pelvis) Orientation
% figure ; plot(t_vec, x_vec(:,4:6)*180/pi) ; grid on ;
%     title('Base (Pelvis) Orientation') ; legend('r','p','y') ;
% 
% % Plot Torques
% figure ; 
%     subplot(2,1,1) ;
%         plot(t_vec, tau_vec(:, [model.jidx.hip_abduction_left; model.jidx.hip_rotation_left; model.jidx.hip_flexion_left; model.jidx.knee_joint_left; model.jidx.toe_joint_left])) ;
%         grid on ; title('Left Torques') ; legend('abduction','rotation','flexion','knee','toe') ;
%     subplot(2,1,2) ;
%         plot(t_vec, tau_vec(:, [model.jidx.hip_abduction_right; model.jidx.hip_rotation_right; model.jidx.hip_flexion_right; model.jidx.knee_joint_right; model.jidx.toe_joint_right])) ;
%         grid on ; title('Right Torques') ; legend('abduction','rotation','flexion','knee','toe') ;
        
%% Animation
% stateData = getVisualizerState(x_vec, model);
% vis = CassieVisualizer(t_vec, stateData);
% 
x1 = x0;
x1(1:20) = q1;



stateData = getVisualizerState(x1', model);
vis = CassieVisualizer([0], stateData);
view([120 0])