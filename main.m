close all;
robot = acrobot.acrobot_anim();
robot.show_plot = 0;

tmax = 6;       % Max simulation time
tstep = 0.01;  % Simulation time step
t = 0;

% Simulate robot falling on the ground
fig = figure;
set(fig, 'Position',  [100, 100, 1500, 700]);

options = odeset('Events',@(t,x)robot.dist_to_floor(t,x), 'RelTol', 1e-9, 'AbsTol', 1e-9);

% Simulate Robot Walking
while (te < tmax)
    
    % Make a step
    [t_anim,x_anim,te,xe] = ode45(@(t, x) robot.step(t, x), t:tstep:tmax, robot.x, options);
    
    % Impact Map (Recalculating state values)
    robot.impact_map(xe);
    
    % Update heel (Update the position of the heel)
    robot.update_heel();
end