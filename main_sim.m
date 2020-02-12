close all;
robot = acrobot.acrobot_control();
% robot.plotHolonomicCurve();

%% Simulate
robot.reset();

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
    
    % Calculate the value for tau at the point
    tau = robot.getTau(robot.x);
    
    % Search for foot placement when close to floor
    [t_anim,x_anim,te,xe, ie] = ode45(@(t, x) robot.step(t, x, tau), [t t+tstep], robot.x, options);

    if (ie)
        robot.impact_foot(xe);
    end
    robot.show(t);
end