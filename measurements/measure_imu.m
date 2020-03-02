%% Arduino Rotary Encoder input
clear; clc;
a = arduino('COM5','Mega2560','Libraries',{'Adafruit/BNO055', 'I2C'});
% dev = i2cdev(a,'0x28')
BNO055Sensor = addon(a,'Adafruit/BNO055');
% % BNO055Sensor2 = addon(a,'Adafruit/BNO055','I2CAddress','0x29');
% 
% 
% 
%% Count Encoder
tic; 
while (toc < 240)
    [status,timestamp] = readCalibrationStatus(BNO055Sensor); 
    status
    if strcmpi(status.Accelerometer,'full') && strcmpi(status.System,'full')
        break; %Accelerometer is calibrated proceed further
    end
    pause(1);
end
while (1)
    [data,timestamp] = readOrientation(BNO055Sensor);
%     pause(1);
    data
end

% while(1)
%     acc = imu.readAcceleration;
%     angvel = imu.readAngularVelocity;
%     
%     fprintf('Acc: %f %f %f Angvel: %f %f %f\n',acc(1), acc(2), acc(3), angvel(1), angvel(2), angvel(3))
%     pause(0.01);
% end