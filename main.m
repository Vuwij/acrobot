%% Robot Setup

clear; clc;close all;
robotParameters;
robot = acrobot.acrobot_control();
estimator = acrobot.acrobot_state_estimator();

tstep = 0.1;  % Time step
rate = rateControl(1/tstep);
%% Device Connection

clear a imu encoder;
a = arduino('COM5','Nano3','BaudRate',115200,'Libraries',{'RotaryEncoder', 'I2C','Adafruit/BNO055'});
% a = arduino('COM5','Mega2560','BaudRate',115200,'Libraries',{'RotaryEncoder', 'I2C','Adafruit/BNO055'});
pause(10);
BNO1  = i2cdev(a,'0x28');
BNO2 = i2cdev(a, '0x29');
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

% read pitch position, we only care about this
[acc1, gyro1, pos1] = read_data(BNO1);
[acc2, gyro2, pos2] = read_data(BNO2);


while (1)
    tic;
    motor_step = encoder.readCount();
    if (motor_step < -100000 || motor_step > 100000)
        encoder.resetCount(last_motor_step);
        motor_step = last_motor_step;
    end
    last_motor_step = motor_step;
    % Get Robot State 
    % gyro2 and acc2 might need to be negated
    state = estimator.stepImplPublic(pos1, gyro1, acc1, -pos2, gyro2, acc2, motor_step);
    
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
    [acc1, gyro1, pos1] = read_data(BNO1);
    [acc2, gyro2, pos2] = read_data(BNO2);
    waitfor(rate);
end


%%
function [acc, gyro, pos] = read_data(BNO)
    x = double(readRegister(BNO,hex2dec('8'),'int16')) / 100.0;
    y = double(readRegister(BNO,hex2dec('A'),'int16')) / 100.0;
    z = double(readRegister(BNO,hex2dec('C'),'int16')) / 100.0;
    acc = [x y z];
    x = double(readRegister(BNO,hex2dec('14'),'int16')) / 16.0;
    y = double(readRegister(BNO,hex2dec('16'),'int16')) / 16.0;
    z = double(readRegister(BNO,hex2dec('18'),'int16')) / 16.0;
    t_gyro = [x y z];
    gyro = convangvel(t_gyro, 'deg/s' ,'rad/s');
    
    deg_pitch = double(readRegister(BNO,hex2dec('1E'),'int16')) / 16.0; % Reads bits 15:8 from register 23
    rad_pitch = deg2rad(deg_pitch);   
    pos = [0 rad_pitch 0];
end