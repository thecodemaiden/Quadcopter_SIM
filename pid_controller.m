function [input, state] = pid_controller(state, motion, target_theta)
    % Initialize integrals to zero when it doesn't exist.
    %Kp = params(:,1); Ki = params(:,2); Kd = params(:,3);
    Kp = state.Kp; Ki = state.Ki; Kd = state.Kd;
    theta = motion.theta;
    thetadot = motion.thetadot;
    if ~isfield(state, 'integral')
        state.lasterror = zeros(3, 1);
        state.integral = zeros(3, 1);
    end

    % Prevent wind-up
    if max(abs(state.integral)) > 0.1
        state.integral(:) = 0;
    end

    % Compute total thrust.
    total = state.m * state.g / state.k / ... 
          (cos(theta(1)) * cos(theta(2)));

    % this will be PI-D controller
      
    now_error = theta-target_theta;
      
    % Compute error and inputs.
    err = Kd .* thetadot + Kp .* now_error + Ki .* state.integral;
    input = err2inputs(state, err, total);

    % Update controller state.
    state.integral = state.integral + state.dt .* now_error;
    state.lasterror = now_error;

end