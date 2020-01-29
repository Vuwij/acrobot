close all;

robot = acrobot.acrobot_control();
robot.show_plot = 1;

tmax = 6;       % Max simulation time
tstep = 0.005;  % Simulation time step
t = 0;

% Simulate robot falling on the ground
fig = figure;
set(fig, 'Position',  [100, 100, 1500, 700]);

% Simulate Robot Walking
while (t < tmax)
    
    % Make a step
    tau = robot.getTau(robot.x);
    dxdt = robot.step(robot.x, tau);
    robot.x = robot.x + dxdt * tstep;
    
    % Alternate foot
    if (robot.dist_to_floor(t,robot.x) < 0)
        robot.impact_foot(robot.x);
    end
    t = t + tstep;
end