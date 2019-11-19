close all;
robot = acrobot.acrobot_anim();
robot.show_plot = 1;

tmax = 6;       % Max simulation time
tstep = 0.005;  % Simulation time step
t = 0;

% Simulate robot falling on the ground
fig = figure;
set(fig, 'Position',  [100, 100, 1500, 700]);

options = odeset('Events',@(t,x)robot.dist_to_floor(t,x), 'RelTol', 1e-9, 'AbsTol', 1e-9);

% Simulate Robot Walking
timeout = 0;
while (t < tmax)
    
    % Make a step
    dxdt = robot.step(t, robot.x);
    robot.x = robot.x + dxdt * tstep;
    
    if (robot.dist_to_floor(t,robot.x) < 0)
        if (timeout == 0)
            % Impact Map (Recalculating state values)
            robot.impact_map(robot.x);

            % Update heel (Update the position of the heel)
            robot.update_heel();
            
            timeout = 30;
        end
    end
    t = t + tstep;
    if (timeout > 0)
        timeout = timeout - 1;
    end
end