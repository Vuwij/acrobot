%% Robot Setup

clear; clc;close all;
robotParameters;
robot = acrobot.acrobot_control();
estimator = acrobot.acrobot_state_estimator();

tstep = 0.05;  % Time step
rate = rateControl(1/tstep);
rotmXYZ = eul2rotm([0 pi 0], 'XYZ');

%% Device Connection

clear a imu encoder;
a = arduino('/dev/ttyUSB0','Nano3','BaudRate',115200,'Libraries',{'RotaryEncoder', 'I2C','Adafruit/BNO055'});
pause(5);
BNO1 = i2cdev(a,'0x28');
BNO2 = i2cdev(a,'0x29');
encoder = rotaryEncoder(a, 'D2','D3', steps_per_rotation);

writeRegister(BNO2,hex2dec('3D'),hex2dec('00'),'uint8');
pause(1);
writeRegister(BNO2,hex2dec('42'),hex2dec('03'),'uint8');
pause(1);
writeRegister(BNO1,hex2dec('3D'),hex2dec('08'),'uint8');
writeRegister(BNO2,hex2dec('3D'),hex2dec('08'),'uint8');

%% Main loop
close all;
fig = figure;
set(fig, 'Position',  [100, 100, 1500, 700]);

estimator.sample_time = tstep;
estimator.setupImplPublic();
encoder.resetCount();

last_motor_step = encoder.readCount();

while (1)
    tic
    
    % State Estimation
    motor_step = encoder.readCount();
    if mod(robot.step_count, 2) == 0
        [acc, ~, pos] = read_data(BNO2);
        pos = -pos;
    else
        [acc, ~, pos] = read_data(BNO1);
        pos(2) = pos(2) + pi;
    end

    [robot.x, collision] = estimator.stepImplPublic(robot.step_count, pos, acc, motor_step);
    if (collision)
        robot.step_count = robot.step_count + 1;
    end
    
    % Control Code
    
    % Motor Output
    
    
    % Display the robot
	robot.plotRobot();

    toc

    % Read next step
    waitfor(rate);
end


%%
function [acc, gyro, pos] = read_data(BNO)
%     x = double(readRegister(BNO,hex2dec('8'),'int16')) / 100.0;
%     y = double(readRegister(BNO,hex2dec('A'),'int16')) / 100.0;
%     z = double(readRegister(BNO,hex2dec('C'),'int16')) / 100.0;
    acc = [0 0 0];
    
%     x = double(readRegister(BNO,hex2dec('14'),'int16')) / 16.0;
%     y = double(readRegister(BNO,hex2dec('16'),'int16')) / 16.0;
%     z = double(readRegister(BNO,hex2dec('18'),'int16')) / 16.0;
%     t_gyro = [x y z];
%     gyro = convangvel(t_gyro, 'deg/s' ,'rad/s');
    gyro = [0 0 0];
    
    deg_pitch = double(readRegister(BNO,hex2dec('1E'),'int16')) / 16.0; % Reads bits 15:8 from register 23
    rad_pitch = deg2rad(deg_pitch);   
    pos = [0 rad_pitch 0];
end