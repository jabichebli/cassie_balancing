function [q, t] = generate_trajectory(q_start, q_des, num_steps)
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
    
    % % Create time vector
    % t = linspace(0, delta_t, num_steps)';
    % 
    % % Normalize time to range [0, 1] to simplify polynomial calculation
    % % tau = t / T
    % tau = t / delta_t;
    tau = linspace(0, 1, num_steps)';
    
    % --- QUADRATIC EASE-OUT ---
    % This is much simpler than quintic.
    % It starts at max speed and decelerates to 0 velocity at the end.
    s = 1 - (1 - tau).^2;
    
    % Calculate Trajectories for all joints
    % q(t) = q0 + (qf - q0) * s(tau)
    delta_q = (qf - q0)'; % Transpose to row vector for broadcasting
    
    q = repmat(q0', num_steps, 1) + (s * delta_q);

    figure(5);
    plot(tau, q, 'LineWidth', 2);
    grid on;
    
    % Labeling
    xlabel('Time (s)');
    ylabel('Position');
    title('Minimum Jerk Trajectory');
    
    % Dynamic Legend Generation
    num_joints = length(q0);
    legend_labels = cell(1, num_joints);
    for i = 1:num_joints
        legend_labels{i} = sprintf('Joint %d', i);
    end
    legend(legend_labels, 'Location', 'best');

end