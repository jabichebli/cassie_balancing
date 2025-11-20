function [q, t] = generate_trajectory(q_start, q_des, delta_t, num_steps)
% GENERATE_TRAJECTORY Generates a minimum-jerk (quintic) trajectory positions.
%
% Inputs:
%   q_start   - Initial joint configuration (Nx1 or 1xN vector)
%   q_des     - Desired/Final joint configuration (Nx1 or 1xN vector)
%   delta_t   - Total duration of movement (seconds)
%   num_steps - Number of discrete time steps
%
% Outputs:
%   q         - Joint Positions (num_steps x N)
%   t         - Time vector (num_steps x 1)

    % Ensure inputs are column vectors for consistent math
    q0 = q_start(:);
    qf = q_des(:);
    
    % Create time vector
    t = linspace(0, delta_t, num_steps)';
    
    % Normalize time to range [0, 1] to simplify polynomial calculation
    % tau = t / T
    tau = t / delta_t;
    
    % Quintic Polynomial Coefficients for rest-to-rest motion:
    % The polynomial is: s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
    % This satisfies s(0)=0, s(1)=1, and 0 velocity/accel at boundaries.
    
    % Position scaling factor
    s = 10 * tau.^3 - 15 * tau.^4 + 6 * tau.^5;
    
    % Calculate Trajectories for all joints
    % q(t) = q0 + (qf - q0) * s(tau)
    delta_q = (qf - q0)'; % Transpose to row vector for broadcasting
    
    q = repmat(q0', num_steps, 1) + (s * delta_q);

end