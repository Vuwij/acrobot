close all;
robot = acrobot.acrobot_control();

%% Simulate
robot.reset();

tmax = 6;       % Max simulation time
tstep = 0.005;  % Simulation time step
t = 0;

% Simulate robot falling on the ground
close all;
fig = figure;
set(fig, 'Position',  [100, 100, 1500, 700]);
options = odeset('Events',@(t,x)robot.dist_to_floor(t,x), 'RelTol', 1e-9, 'AbsTol', 1e-9);

% Simulate Robot Walking
while (t < tmax)
    
    % Calculate the value for tau at the point
    tau = robot.getTau(robot.x);
    
    % Test
%     tau = [0;0.00];
% 
%     if (robot.step_count == 0)
%         tau = [0;0.00];
%     elseif (mod(robot.step_count,2) == 1)
%         tau = [0;robot.c2.tau_const];
%     else
%         tau = [0;robot.c1.tau_const];
%     end
    
    % Search for foot placement when close to floor
    t_next = floor((t + tstep + 1e-9)/tstep)*tstep;
    [t_anim, x_anim, te, xe, ie] = ode45(@(t, x) robot.step(t, x, tau), [t t_next], robot.x, options);
    
    if (ie)
        robot.impact_foot(xe);
        t = te;
    else
        robot.x = x_anim(end,:)';
        t = t_next;
    end
    
    % End Conditions
    if (robot.x(2) > pi || robot.x(2) < -pi)
        disp("Robot Impacted With Itself");
        break
    end
    
    robot.show(t);
end