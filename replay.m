robotParameters;
robot = acrobot.acrobot_control();

slowdown = 0.1;
tstep = 0.05 / slowdown;  % Time step
rate = rateControl(1/tstep);

close all;
fig = figure;
set(fig, 'Position',  [100, 100, 1500, 700]);
load('data/tests/test_03-12-2020 23-23.mat');
for t = ts.Time'
    robot.x = ts.getsampleusingtime(t).Data;
    tau = robot.getTau(robot.x);
    
    robot.show(t);
    waitfor(rate);
end