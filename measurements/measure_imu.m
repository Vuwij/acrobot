%% IMU Testing
clear; clc;
a = arduino('/dev/ttyUSB0','Nano3','Libraries','I2C');
imu = mpu6050(a);


%% Count Encoder
FUSE = imufilter();
while(1)
    
    acc = imu.readAcceleration;
    angvel = imu.readAngularVelocity;
    [orient,angularVelocity] = FUSE(acc,angvel);
    zyx = eulerd(orient,'ZYX','frame');
    zyx = deg2rad(zyx);
    fprintf('angle: %f %f %f\n', zyx(1), zyx(2), zyx(3));
%     fprintf('Acc: %f %f %f Angvel: %f %f %f\n',acc(1), acc(2), acc(3), angvel(1), angvel(2), angvel(3))
    pause(0.5);
end