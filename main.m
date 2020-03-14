%% Robot Setup

clear; clc;close all;
robotParameters;
robot = acrobot.acrobot_control();
torque_controller = acrobot.acrobot_torque_control();
estimator = acrobot.acrobot_state_estimator();

tstep = 0.035;  % Time step
rate = rateControl(1/tstep);
rotmXYZ = eul2rotm([0 pi 0], 'XYZ');

%% Device Connection

clear a imu encoder BNO1 BNO2;
a = arduino('/dev/ttyUSB0','Nano3','BaudRate',115200,'Libraries',{'RotaryEncoder', 'I2C','Adafruit/BNO055'});
writeDigitalPin(a, 'D6', 0);
writeDigitalPin(a, 'D7', 1);
writePWMDutyCycle(a,'D9',0);
pause(2);
BNO1 = i2cdev(a,'0x28');
BNO2 = i2cdev(a,'0x29');
encoder = rotaryEncoder(a, 'D2','D3', steps_per_rotation);

writeRegister(BNO2,hex2dec('3F'), hex2dec('20'),'uint8');
pause(1);
writeRegister(BNO2,hex2dec('3D'),hex2dec('00'),'uint8');
writeRegister(BNO1,hex2dec('3D'),hex2dec('00'),'uint8');
pause(1);

writeRegister(BNO2,hex2dec('42'),hex2dec('03'),'uint8');
writeRegister(BNO1,hex2dec('42'),hex2dec('03'),'uint8');
pause(1);
writeRegister(BNO1,hex2dec('3D'),hex2dec('08'),'uint8');
writeRegister(BNO2,hex2dec('3D'),hex2dec('08'),'uint8');

%% Main loop
close all;
test_state_estimation = 0;

if test_state_estimation
    fig = figure;
    set(fig, 'Position',  [100, 100, 1500, 700]);
end

estimator.sample_time = tstep;
estimator.setupImplPublic();
encoder.resetCount();
robot.reset();
robot.gamma = 2;
last_motor_step = encoder.readCount();
t = 0;
duration = 20;
ts = timeseries('acrobot_data');

BN01_offset = 0;
BN02_offset = 1.1494 - pi/2;
pwm = 0;
try
    while (t < duration)
        tic

        % State Estimation
        motor_step = encoder.readCount();
        if mod(robot.step_count, 2) == 0
            [acc, ~, pos] = read_data(BNO2);
            pos = wrapToPi(-pos + BN02_offset);
        else
            [acc, ~, pos] = read_data(BNO1);
            pos = wrapToPi(pos + BN01_offset);
        end
        
        [robot.x, collision] = estimator.stepImplPublic(robot.step_count, pos, acc, motor_step);
        if (collision)
            robot.step_count = robot.step_count + 1;
        end
        

        % Control Code
        tau = robot.getTau(robot.x);
        
        
        % End Conditions
        if (robot.x(2) > pi - robot.angle_limit || robot.x(2) < -pi + robot.angle_limit || robot.x(1) > pi || robot.x(1) < 0)
            disp(strcat("Robot Impacted With Itself, angles: x1: ", num2str(robot.x(1)), " x2: ", num2str(robot.x(2))));
            break
        end

        
        % Motor Output
        pwm_new = torque_controller.getPWM(tau(2));
        if (pwm_new < 0 && pwm >= 0)
            writeDigitalPin(a, 'D6', 1);
            writeDigitalPin(a, 'D7', 0);
        elseif (pwm_new > 0 && pwm <= 0)
            writeDigitalPin(a, 'D6', 0);
            writeDigitalPin(a, 'D7', 1);
        end
        pwm = pwm_new;
        if test_state_estimation
            robot.show(t);
        else
            writePWMDutyCycle(a,'D9',abs(pwm));
        end
        
        % Display the robot
        ts = ts.addsample('Data',robot.x,'Time',t);
        tend = toc;
        fprintf("Elapsed Time: %.3f\t x1: %.3f\t x2: %.3f\t tau:%.3f\t pwm: %.3f\n", tend, robot.x(1), robot.x(2), tau(2), pwm);

        % Read next step
        waitfor(rate);
        t = t + tstep;
    end
catch ex
    disp(ex)
end
writeDigitalPin(a, 'D6', 0);
writeDigitalPin(a, 'D7', 1);
writePWMDutyCycle(a,'D9',0);

filename = sprintf('data/tests/test_%s', datestr(now,'mm-dd-yyyy HH-MM'));
save(filename, 'ts')

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