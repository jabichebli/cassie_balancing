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
% dq = x0(model.n+1 : 2*model.n);

persist_params.foot_dx = 0.3;
persist_params.foot_dy = 0.3;

q(3) = q(3)-0.05;

foot_des = [[0.0921;0.1305;0],...
         [-0.0879;0.1305;0],...
         [0.0921+persist_params.foot_dx;-0.1305-persist_params.foot_dy;0.1],...
         [-0.0879+persist_params.foot_dx;-0.1305-persist_params.foot_dy;0.1]];

% p_target_ankle = [-0.187033071501011;-0.601072567829665;0.106247970684128+0.05];
% p_target_toe = [-0.362003136439526;-0.581710202000642;0.143805595275082+0.05];

p_target_ankle = [-0.2052;-0.6278;0.1047+0.05];
p_target_toe = [-0.3850;-0.6244;0.1106+0.05];

q2 = q;

q1 = solveFootIK(model, foot_des(:,1), foot_des(:,2), foot_des(:,3), foot_des(:,4), [1;1;0;0], q);

% q2 = solveSingleFootIK(model, 'world', 'left', foot_des(:,1), foot_des(:,2), q2);
% q2 = solveSingleFootIK(model, 'body', 'right', p_target_ankle, p_target_toe, q2);

num_traj = 1;
traj = generate_trajectory(q, q1, num_traj);

q_arr = traj;

t_space = linspace(1,1,num_traj);

x1_arr = zeros([48, length(t_space)+1]);
x1_arr(:,end) = x0; % Initialize the last column of x1_arr with the initial state

% for t = 1:length(t_space)
% 
%     x1 = x0;
%     x1(1:20) = q_arr(t,:);
%     x1_arr(:,t) = x1;
% 
% end

x1 = x0;
x1(1:20) = q1;

x1(1:20) = [
    0.000000000000015;
    0;
    1.000580162074814;
    0;
    0;
    0;
    0.156039996234321;
    -0.165107662954847;
    0.000254254777004;
    0.000000003892431;
    0.038292224543740;
    1.240086598509995;
    -0.664991466524112;
    -2.149281713216409;
    0;
    0;
    1.426892802759263;
    1.426892802759263;
    -1.752079511258373;
    -1.390362051838107
];

% disp('one shot')
% [p1, p2, p3, p4] = computeFootPositions(q1, model)
% disp('two shots')
% [p1, p2, p3, p4] = computeFootPositions(q2, model)

% stateData = getVisualizerState(x1_arr', model);

stateData = getVisualizerState(x1', model);
vis = CassieVisualizer([0], stateData);
disp("end")