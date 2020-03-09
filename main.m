%% Robot Setup

clear; clc;close all;
robotParameters;
robot = acrobot.acrobot_control();
estimator = acrobot.acrobot_state_estimator();

tstep = 0.1;  % Time step
rate = rateControl(1/tstep);
%% Device Connection

clear a imu encoder;
a = arduino('COM3','Nano3','BaudRate',115200,'Libraries',{'RotaryEncoder', 'I2C','Adafruit/BNO055'});
% a = arduino('COM5','Mega2560','BaudRate',115200,'Libraries',{'RotaryEncoder', 'I2C','Adafruit/BNO055'});
pause(10);
BNO055Sensor = addon(a,'Adafruit/BNO055','I2CAddress','0x28');
% BNO055Sensor2 = addon(a,'Adafruit/BNO055');
encoder = rotaryEncoder(a, 'D2','D3', steps_per_rotation);

%% Main loop
close all;
fig = figure;
set(fig, 'Position',  [100, 100, 1500, 700]);

estimator.sample_time = tstep;
estimator.setupImplPublic();
encoder.resetCount();

last_motor_step = encoder.readCount();
test = 0;

% Calibration for sensor, probably need to calibrate them separately
tic;
while (toc < 240)
    [status,timestamp] = readCalibrationStatus(BNO055Sensor); 
    status
    if strcmpi(status.Accelerometer,'full') && strcmpi(status.System,'full') && strcmpi(status.Gyroscope,'full')
        break; %Accelerometer is calibrated proceed further
    end
    pause(1);
end

% [acc, gyro, ts, overrun] = imu.read();
[acc1,timestamp] = readAcceleration(BNO055Sensor);
[gyro1,timestamp] = readAngularVelocity(BNO055Sensor);
[pos1, timestamp] = readOrientation(BNO055Sensor);
while (1)
    tic;
    motor_step = encoder.readCount();
    if (motor_step < -100000 || motor_step > 100000)
        encoder.resetCount(last_motor_step);
        motor_step = last_motor_step;
    end
    last_motor_step = motor_step;
    
    % Get Robot State (Fix this line)
    state = estimator.stepImplPublic(pos1, gyro1, acc1, pos1, gyro1, acc1, motor_step);
    
    % Update the robot state with the estimated state (Might want to tune
    % it so that it takes a percentage of the measured vs a percentage of
    % the projected state. Can use a complementary filter for now but can
    % upgrade to kalman filter sometime in the future
    robot.x = state;
%     robot.x
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
    [acc1,timestamp] = readAcceleration(BNO055Sensor);
    [gyro1,timestamp] = readAngularVelocity(BNO055Sensor);
    [pos1, timestamp] = readOrientation(BNO055Sensor);
    waitfor(rate);
end