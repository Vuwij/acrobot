robotParameters;
if exist('robot', 'var')
    robot.reset();
else
    robot = acrobot.acrobot_control();
end

slowdown = 5.0;
tstep = 0.07 / slowdown;  % Time step
rate = rateControl(1/tstep);

close all;
fig = figure;
set(fig, 'Position',  [100, 100, 1500, 700]);
listing = dir('data/tests/');
load(strcat('data/tests/',listing(end).name));
for t = ts.Time'
    robot.x = ts.getsampleusingtime(t).Data;
    tau = robot.getTau(robot.x);
    
    robot.show(t);
    waitfor(rate);
end