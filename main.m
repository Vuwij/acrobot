close all;
robot = acrobot.acrobot_control();
% figure;
% robot.plotHolonomicCurve();
robot.show_plot = 1;

tmax = 6;       % Max simulation time
tstep = 0.005;  % Simulation time step
t = 0;

% Simulate robot falling on the ground
fig = figure;
set(fig, 'Position',  [100, 100, 1500, 700]);

% Collision point on the floor
search_for_collision_threshold = 0.05;
options = odeset('Events',@(t,x)robot.dist_to_floor(t,x), 'RelTol', 1e-9, 'AbsTol', 1e-9);

% Simulate Robot Walking
while (t < tmax)
    
    % Make a step
    dxdt = robot.step(t, robot.x);
    robot.x = robot.x + dxdt * tstep;
    
    robot.show(t);
    
    % Search for foot placement when close to floor
    if (robot.dist_to_floor(t,robot.x) < search_for_collision_threshold && ((robot.x(2) < 0 && robot.x(4) < 0)))
        [t_anim,x_anim,t,xe] = ode45(@(t, x) robot.autostep(t, x), t:tstep:tmax, robot.x, options);
        
        % Impact Map and replace feet
        robot.impact_foot(xe)
    end
    t = t + tstep;
end