clear; clc;
a = arduino('COM7','Nano3','Libraries',{'RotaryEncoder', 'I2C'});
encoder = rotaryEncoder(a,'D2','D3');

while(1)
    count = readCount(encoder);
%     fprintf('Count: %d\n', count);
    measuredAngle = mod(((360 / 2797) * count), 360); 
    fprintf('%d\n', (360 / 2797) * count);
%     fprintf('Current knob angle: %d\n', measuredAngle);
    pause(0.001);
end