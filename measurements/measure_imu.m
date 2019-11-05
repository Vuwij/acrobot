%% Arduino Rotary Encoder input
clear; clc;
a = arduino('/dev/ttyUSB0','Nano3','Libraries','I2C');
imu = mpu6050(a);

%% Count Encoder
while(1)
    acc = imu.readAcceleration;
    angvel = imu.readAngularVelocity;
    
    fprintf('Acc: %f %f %f Angvel: %f %f %f\n',acc(1), acc(2), acc(3), angvel(1), angvel(2), angvel(3))
    pause(0.01);
end