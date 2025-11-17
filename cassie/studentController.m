function tau = studentController(t, s, model, ~)
% Modify this code to calculate the joint torques
% t - time
% s - state of the robot
% model - struct containing robot properties
% params - user defined parameters in studentParams.m
% tau - 10x1 vector of joint torques

% State vector components ID
q = s(1 : model.n);
dq = s(model.n+1 : 2*model.n);

%% [Control #1] zero control
% tau = zeros(10,1);

%% [Control #2] Highest Score so far on leaderboard 

% Leaderboard score  |  Kp  |  Kd
%      218.64        | 1800 |  300
%      218.37        | 3000 |  500
%      163.67        | 1200 |  200 

% kp = 1800 ; 
% kd = 300 ;
% x0 = getInitialState(model);
% q0 = x0(1:model.n) ;
% tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx) ;


%% [Control #2.1] 
% % Score: 218.64
% 
% % -------------------------------------------------------------------------
% % ------------------------- Posture Controller ----------------------------
% % -------------------------------------------------------------------------
% 
% % Define the gains
% kp_posture = 1800 ; 
% kd_posture = 300 ;
% 
% % Get initial state for the cassie robot
% x0 = getInitialState(model);
% q0 = x0(1:model.n) ;
% 
% % Calculate the posture-keeping torques
% tau_posture = -kp_posture*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd_posture*dq(model.actuated_idx) ;
% 
% % -------------------------------------------------------------------------
% % --------------------------- COM Controller ------------------------------
% % -------------------------------------------------------------------------
% 
% % -------------------------- Current Position -----------------------------
% % Get the current 3D position and velocity of the COM
% [r_com, v_com] = computeComPosVel(q, dq, model);
% 
% % Get current horizontal (x,y) COM state
% r_com_xy = r_com(1:2);
% v_com_xy = v_com(1:2);
% 
% % Get the 3D positions of the four contact points on the feet
% [p1, p2, p3, p4] = computeFootPositions(q, model);
% 
% % -------------------------- Desired Position -----------------------------
% 
% % Define the support polygon 
% rd_com_x = (p1(1) + p2(1) + p3(1) + p4(1)) / 4;
% rd_com_y = (p1(2) + p2(2) + p3(2) + p4(2)) / 4;
% 
% % Desired horizontal (x,y) COM position is the support center
% rd_com_xy = [rd_com_x; rd_com_y];
% 
% % Desired horizontal (x,y) COM velocity is zero
% vd_com_xy = [0; 0];
% 
% % --------------- Calculate Desired Horizontal Force at COM ---------------
% % Define gains
% kp_com = 2000;
% kd_com = 100;
% 
% % Use a PD controller to calculate the desired force to apply
% fd_xy = -kp_com * (r_com_xy - rd_com_xy) - kd_com * (v_com_xy - vd_com_xy);
% 
% % ------------------ Map Desired Force to Joint Torques -------------------
% % Jacobian Transpose Method
% 
% % Get the full COM Jacobian (maps all joint vels to 3D COM vel)
% J_com = computeComJacobian(q, model); 
% 
% % Get the part of the Jacobian for only the horizontal (x,y) COM and only the actuated joints
% J_com_xy_act = J_com(1:2, model.actuated_idx);
% 
% % Calculate the corrective torques
% tau_com = J_com_xy_act' * fd_xy;
% 
% % -------------------------------------------------------------------------
% % ------------------------ Combine Controllers ----------------------------
% % -------------------------------------------------------------------------
% 
% % Combine the posture and COM control torques
% tau = tau_posture + tau_com;


%% [Control #2.2] 
% Score: 

% -------------------------------------------------------------------------
% ------------------------- Posture Controller ----------------------------
% -------------------------------------------------------------------------

% Define the gains
kp_posture = 1800; % 1000
kd_posture = 300; % 400 

% Get initial state for the cassie robot
x0 = getInitialState(model);
q0 = x0(1:model.n) ;

% Calculate the posture-keeping torques
tau_posture = -kp_posture*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd_posture*dq(model.actuated_idx) ;

% -------------------------------------------------------------------------
% --------------------------- COM Controller ------------------------------
% -------------------------------------------------------------------------

% -------------------------- Current Position -----------------------------
% Get the current 3D position and velocity of the COM
[r_com, v_com] = computeComPosVel(q, dq, model);

% Get current horizontal (x,y) COM state
r_com_xy = r_com(1:2);
v_com_xy = v_com(1:2);

% Get the 3D positions of the four contact points on the feet
[p1, p2, p3, p4] = computeFootPositions(q, model);

% --------------------------- Suport Polygon ------------------------------

% Define the support polygon vertices
poly_points_x = [p1(1), p2(1), p3(1), p4(1)];
poly_points_y = [p1(2), p2(2), p3(2), p4(2)];

% Find the convex hull to get the vertices in order
k = convhull(poly_points_x, poly_points_y);
poly_x = poly_points_x(k);
poly_y = poly_points_y(k);

% Check if the current COM is inside or on the polygon
[in, on] = inpolygon(r_com_xy(1), r_com_xy(2), poly_x, poly_y);

% --------------- Calculate Desired Horizontal Force at COM ---------------
% Define gains
kp_com = 2000; 
kd_com = 500;

if in || on
    % --- COM is INSIDE the stable region ---
    % The robot is safe.
    % Target is current position (damping only)
    rd_com_xy = r_com_xy;
    vd_com_xy = [0; 0];
    
    fd_xy = -kp_com * (r_com_xy - rd_com_xy) - kd_com * (v_com_xy - vd_com_xy);

else
    % --- COM is OUTSIDE the stable region ---
    % Implement your "halfway-to-center" logic
    
    % 1. Find the center
    rd_center_x = (p1(1) + p2(1) + p3(1) + p4(1)) / 4;
    rd_center_y = (p1(2) + p2(2) + p3(2) + p4(2)) / 4;
    rd_center = [rd_center_x; rd_center_y];

    % 2. Find the closest point on the polygon boundary (p_edge)
    num_verts = length(poly_x);
    min_dist_sq = inf;
    p_edge = [0; 0]; % This will store the closest point
    
    for i = 1:num_verts
        % Define the edge segment (a to b)
        a = [poly_x(i); poly_y(i)];
        if i < num_verts
            b = [poly_x(i+1); poly_y(i+1)];
        else
            b = [poly_x(1); poly_y(1)]; % Wrap around to close the loop
        end
        
        % Math to find the closest point on a line segment
        ap = r_com_xy - a; % Vector from a to our COM
        ab = b - a;       % Vector for the edge
        
        t = (ap' * ab) / (ab' * ab);
        t = max(0, min(1, t)); % Clamp t to be on the segment [0, 1]
        
        closest_point_on_segment = a + t * ab;
        
        % Check if this is the new closest point
        dist_sq = (r_com_xy - closest_point_on_segment)' * (r_com_xy - closest_point_on_segment);
        
        if dist_sq < min_dist_sq
            min_dist_sq = dist_sq;
            p_edge = closest_point_on_segment;
        end
    end

    % 3. Define the new desired COM position
    % Your idea: Target is halfway between the center and the closest edge
    % rd_com_xy = (rd_center + p_edge) / 2;
    rd_com_xy = rd_center;
    vd_com_xy = [0; 0]; % Desired velocity is still zero
    
    % 4. Calculate the PD force to push to this new target
    fd_xy = -kp_com * (r_com_xy - rd_com_xy) - kd_com * (v_com_xy - vd_com_xy);
end

% ------------------ Map Desired Force to Joint Torques -------------------
% Jacobian Transpose Method

% Get the full COM Jacobian (maps all joint vels to 3D COM vel)
J_com = computeComJacobian(q, model); 

% Get the part of the Jacobian for only the horizontal (x,y) COM and only the actuated joints
J_com_xy_act = J_com(1:2, model.actuated_idx);

% Calculate the corrective torques
tau_com = J_com_xy_act' * fd_xy;

% -------------------------------------------------------------------------
% ------------------------ Combine Controllers ----------------------------
% -------------------------------------------------------------------------

% Combine the posture and COM control torques
tau = tau_posture + tau_com;


%% [Control #3] Force Controller

% % Get initial conditons 
% x0 = getInitialState(model);
% q0 = x0(1:model.n) ;
% dq0 = zeros(model.n, 1);
% 
% kp_fd = 1800 ; 
% kd_fd = 300 ;
% kp_td = 0;
% kd_td = 0;
% g = 9.81; 
% 
% [rd_com, vd_com] = computeComPosVel(q0, dq0, model);
% [r_com, v_com] = computeComPosVel(q, dq, model);
% 
% fd = - kp_fd * (r_com - rd_com) - kd_fd * (v_com - vd_com) + model.M * g * [0; 0; 1];
% 
% 
% e_orient = zeros(length(fd),1);
% e_ang_vel = zeros(length(fd),1);
% 
% taud = - kp_td * e_orient - kd_td * e_ang_vel;
% 
% Fd = [fd; taud];
% 
% [p1, p2, p3, p4] = computeFootPositions(q, model);


% reaction forces constraints



% friction cone constraints



end
