%% Arduino Rotary Encoder input
clear; clc;
a = arduino('COM4','Nano3','Libraries',{'Adafruit/BNO055', 'I2C'});
BNO055Sensor = addon(a,'Adafruit/BNO055', 'I2CAddress', '0x28','Bus',0);
BNO055Sensor2 = addon(a,'Adafruit/BNO055','I2CAddress','0x29','Bus',0);

%%

c = 5;
tstep = 0.1;  % Time step
rate = rateControl(1/tstep);

while (1)
    [data,timestamp] = readAcceleration(BNO055Sensor);
    [data2,timestamp] = readAcceleration(BNO055Sensor2);
    
    data
    data2
    waitfor(rate);
end

