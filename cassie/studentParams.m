function params = studentParams(model)
% define any parameters here 
% params - struct

params = struct();

params.foot_dx = -0.1;
params.foot_dy = 0.2;

% params.stage1 = [[0.0921+foot_dx;0.1305+foot_dy;0.05], [-0.0879+foot_dx;0.1305+foot_dy;0.05], [0.0921;-0.1305;0], [-0.0879;-0.1305;0]];
% % params.stage1 = [[0.0921;0.1305;-0.2], [-0.0879;0.1305;0-0.2], [0.0921;-0.1305;-0.2], [-0.0879;-0.1305;-0.2]];
% params.stage2 = [[0.0921;0.1305+foot_dy*2;0], [-0.0879;0.1305+foot_dy*2;0], [0.0921;-0.1305;0], [-0.0879;-0.1305;0]];

% params.q1 = solveFootIK(model, [0.0921;0.1305;0.1], [-0.0879;0.1305;0.1], [0.0921;-0.1305;0], [-0.0879;-0.1305;0], q);
params.t1 = 0.4;

% params.q2 = solveFootIK(model, [0.0921;0.405;0], [-0.0879;0.405;0], [0.0921;-0.1305;0], [-0.0879;-0.1305;0], q);
params.t2 = 0.5;

params.tolerance = 0.05;

end