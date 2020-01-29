%% Connection to the robot
clear; clc;close all;
a = arduino('/dev/ttyUSB0','Nano3','Libraries',{'RotaryEncoder', 'I2C'});
encoder = rotaryEncoder(a,'D2','D3');
imu = mpu6050(a);

%% Main loop
robot = acrobot.acrobot_control();
estimator = acrobot.acrobot_state_estimator();
robot.show_plot = 1;

tstep = 0.005;  % Time step
rate = rateControl(1/tstep);

% Simulate robot falling on the ground
fig = figure;
set(fig, 'Position',  [100, 100, 1500, 700]);

% Simulate Robot Walking
while (1)
    
    % Get Robot State (Fix this line)
    state = estimator.stepImpl(obj, gyro, acc, motor_step);
    
    % Update the robot state with the estimated state (Might want to tune
    % it so that it takes a percentage of the measured vs a percentage of
    % the projected state. Can use a complementary filter for now but can
    % upgrade to kalman filter sometime in the future
    robot.x = state;
    
    % Make a step and calculate tau
    tau = robot.getTau(robot.x);
    dxdt = robot.step(robot.x, tau); 
    % robot.x = robot.x + dxdt * tstep; % Do not update robot state for now
    
    % TODO: send the tau to the actual robot (Miguel)
    
    % Alternate foot
    if (robot.dist_to_floor(t,robot.x) < 0)
        robot.impact_foot(robot.x);
    end
    
    % Wait for the rate to finish
    waitfor(rate);
end