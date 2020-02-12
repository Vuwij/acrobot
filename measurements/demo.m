%% Arduino Rotary Encoder input
clear; clc;
a = arduino('/dev/ttyUSB0','Nano3','Libraries',{'RotaryEncoder', 'I2C'});
encoder = rotaryEncoder(a,'D2','D3');
imu = mpu6050(a);
rate = rateControl(100);
%% Count Encoder

while(1)
    acc = imu.readAcceleration;
    angvel = imu.readAngularVelocity;
    
    
    val = atan2(acc(2), acc(3));
    if (val > 0)
        writeDigitalPin(a, 'D6', 1);
        writeDigitalPin(a, 'D7', 0);
    else
        writeDigitalPin(a, 'D6', 0);
        writeDigitalPin(a, 'D7', 1);
    end
    
    power = min(abs(val)/2,1);
    writePWMDutyCycle(a,'D9',power);
    
    fprintf('Acc: %f %f %f Angvel: %f %f %f Power: %f\n',acc(1), acc(2), acc(3), angvel(1), angvel(2), angvel(3), power)

    waitfor(rate);
end