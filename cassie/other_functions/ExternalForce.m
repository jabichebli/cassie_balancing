function f_ext = ExternalForce(t, q,model)
% F_pert 6x1 - roll, pitch, yaw, x,y,z
% if t > 0 && t < 0.1
%     F_pert = [10; 10; 10; 10; 10; 10];
% else
%     F_pert = [0; 0; 0; 0; 0; 0];
% end
F_pert = [0 0 0 0 0 0]';
 % F_pert = [0; 0; 0; 0; 0; 0];

% apply perturbation force on torso
f_ext = cell(1,model.NB);
bXw_curr = bodypos(model, model.idx.torso, q) ;
f_ext{model.idx.torso} = bXw_curr' * F_pert;
    