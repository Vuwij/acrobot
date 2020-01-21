%% Arduino Rotary Encoder input
clear; clc;
a = arduino('COM4','Nano3','Libraries',{'RotaryEncoder', 'I2C'});
encoder = rotaryEncoder(a,'D2','D3');
imu = mpu6050(a);

%% Count Encoder

while(1)
    acc = imu.readAcceleration;
    angvel = imu.readAngularVelocity;
    
    fprintf('Acc: %f %f %f Angvel: %f %f %f\n',acc(1), acc(2), acc(3), angvel(1), angvel(2), angvel(3))
    
    val = acc(3)/10;
    if (val > 1)
        val = 1;
        writeDigitalPin(a, 'D6', 1);
        writeDigitalPin(a, 'D7', 0);
        writeDigitalPin(a, 'D8', 1);
        writeDigitalPin(a, 'D9', 0);
    elseif (val < 0)
        if (val < -1)
            val = -1;
        end
        val = -val;
        writeDigitalPin(a, 'D6', 0);
        writeDigitalPin(a, 'D7', 1);
        writeDigitalPin(a, 'D8', 0);
        writeDigitalPin(a, 'D9', 1);
    end
    
    writePWMDutyCycle(a,'D5',val);
    
    pause(0.01);
end