%% Arduino Rotary Encoder input
clear; clc;
a = arduino('COM5','Nano3','Libraries',{'Adafruit/BNO055', 'I2C'});
% BNO055Sensor = addon(a,'Adafruit/BNO055', 'I2CAddress', '0x28');
% BNO055Sensor2 = addon(a,'Adafruit/BNO055','I2CAddress','0x29');
pause(10);
% BNO1  = i2cdev(a,'0x28');
BNO2 = i2cdev(a, '0x29');
%%
writeRegister(BNO2,hex2dec('3D'),hex2dec('00'),'uint8');
pause(5);
% a1 = readRegister(BNO2,hex2dec('3D'),'uint8');
% c1 = bitget(a1,8:-1:1)
writeRegister(BNO2,hex2dec('42'),hex2dec('05'),'uint8');
a1 = readRegister(BNO2,hex2dec('42'),'uint8');
b1 = bitget(a1,8:-1:1)
writeRegister(BNO2,hex2dec('3D'),hex2dec('08'),'uint8');
pause(5);
%%
while (1)
    deg_pitch = double(readRegister(BNO2,hex2dec('1E'),'int16')) / 16.0 % Reads bits 15:8 from register 23
    rad_pitch = deg2rad(deg_pitch);   
    pos = [0 rad_pitch 0];
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
