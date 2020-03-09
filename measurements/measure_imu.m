%% Arduino Rotary Encoder input
clear; clc;
a = arduino('COM4','Nano3','Libraries',{'Adafruit/BNO055', 'I2C'});
% BNO055Sensor = addon(a,'Adafruit/BNO055', 'I2CAddress', '0x28');
% BNO055Sensor2 = addon(a,'Adafruit/BNO055','I2CAddress','0x29');
BNO1  = i2cdev(a,'0x28');
BNO2 = i2cdev(a, '0x29');


%%
while (1)
    pitch1 = double(readRegister(BNO1,hex2dec('1E'),'int16')) / 16.0; % Reads bits 15:8 from register 23
    pos1 = [0 deg2rad(pitch1) 0]
%     pitch2 = double(readRegister(BNO2,hex2dec('1E'),'int16')) / 16.0; % Reads bits 15:8 from register 23
%     pos2 = [0 deg2rad(pitch2) 0]

end
%%

% c = 5;
% tstep = 0.1;  % Time step
% rate = rateControl(1/tstep);
% 
% while (1)
%     [data,timestamp] = readOrientation(BNO055Sensor);
%     [data2,timestamp] = readAcceleration(BNO055Sensor2);
%     
%     datareadRegister(BNO, 23)
%     data2
%     waitfor(rate);
% end
%%
function acc = read_acc(BNO)
    x = double(readRegister(BNO,hex2dec('8'),'int16')) / 100.0;
    y = double(readRegister(BNO,hex2dec('A'),'int16')) / 100.0;
    z = double(readRegister(BNO,hex2dec('C'),'int16')) / 100.0;
    acc = [x y z];
end
