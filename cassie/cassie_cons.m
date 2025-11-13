function [cineq, ceq] = cassie_cons(x, params)
   
    ceq = [];
    cineq = [];

    %% Get initial conditons 
    x0 = getInitialState(model);
    q0 = x0(1:model.n) ;
    dq0 = zeros(model.n, 1);
    
    kp_fd = 1800 ; 
    kd_fd = 300 ;
    kp_td = 0;
    kd_td = 0;
    g = 9.81; 
    
    [rd_com, vd_com] = computeComPosVel(q0, dq0, model);
    [r_com, v_com] = computeComPosVel(q, dq, model);
    
    fd = - kp_fd * (r_com - rd_com) - kd_fd * (v_com - vd_com) + model.M * g * [0; 0; 1];
    
    
    e_orient = zeros(length(fd),1);
    e_ang_vel = zeros(length(fd),1);
    
    taud = - kp_td * e_orient - kd_td * e_ang_vel;
    
    Fd = [fd; taud];
    
    [p1, p2, p3, p4] = computeFootPositions(q, model);

    %% (a) F_c constraints

    f1 = x(1); f2 = x(2); f3 = x(3); f4 = x(4);
    Fc = cross(p1, f1) + cross(p2, f2) + cross(p3, f3) + cross(p4, f4);
    ceq = [ceq; Fc - Fd];

    %% (b) reaction force constraint

    F=[f1;f2;f3;f4];
    cineq = [cineq; F * [0;0;1]];
   

    % %% (b) Friction cone: max( |Fh(t)| - mu_s * Fv(t) ) <= 0
    % friction_violation = abs(Fst_h) - mu_s * Fst_v;
    % cineq_fric = max(friction_violation);
    % 
    % 
    % % Combine all inequality constraints into one vector
    % cineq = [cineq_Fv; cineq_fric; cineq_speed];
end