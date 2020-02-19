%% Robot Setup

clear; clc;close all;
robotParameters;
robot = acrobot.acrobot_control();
estimator = acrobot.acrobot_state_estimator();

tstep = 0.1;  % Time step
rate = rateControl(1/tstep);
%% Device Connection

clear a imu encoder;
a = arduino('/dev/ttyUSB0','Nano3','BaudRate',115200,'Libraries',{'RotaryEncoder', 'I2C'});
imu = mpu6050(a,'SampleRate',50,'SamplesPerRead',1,'OutputFormat','matrix');
encoder = rotaryEncoder(a, 'D2','D3', steps_per_rotation);

%% Main loop
close all;
fig = figure;
set(fig, 'Position',  [100, 100, 1500, 700]);

estimator.sample_time = tstep;
estimator.setupImplPublic();
encoder.resetCount();

[acc, gyro, ts, overrun] = imu.read();
last_motor_step = encoder.readCount();
test = 0;
while (1)
    tic;
    motor_step = encoder.readCount();
    if (motor_step < -100000 || motor_step > 100000)
        encoder.resetCount(last_motor_step);
        motor_step = last_motor_step;
    end
    last_motor_step = motor_step
    
    % Get Robot State (Fix this line)
    gyro
    state = estimator.stepImplPublic(gyro', acc', motor_step);
    
    % Update the robot state with the estimated state (Might want to tune
    % it so that it takes a percentage of the measured vs a percentage of
    % the projected state. Can use a complementary filter for now but can
    % upgrade to kalman filter sometime in the future
    robot.x = state;
    
    % Make a step and calculate tau
%    tau = robot.getTau(robot.x);
%    dxdt = robot.step(robot.x, tau); 
    % robot.x = robot.x + dxdt * tstep; % Do not update robot state for now
    
    % TODO: send the tau to the actual robot (Miguel)
    
    % Alternate foot
%     if (robot.dist_to_floor(t,robot.x) < 0)
%         robot.impact_foot(robot.x);
%     end
    
    % Display the robot
    robot.plotRobot();
    
    % Read next step
    [acc, gyro, ts, overrun] = imu.read();
    waitfor(rate);
end