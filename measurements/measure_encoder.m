%% Arduino Rotary Encoder input
clear; clc;
a = arduino('/dev/ttyUSB0','Nano3','Libraries','RotaryEncoder');

%% Count Encoder
clear encoder
encoder = rotaryEncoder(a,'D2','D3');

% Forward
writeDigitalPin(a, 'D6', 1);
writeDigitalPin(a, 'D7', 0);
writeDigitalPin(a, 'D8', 1);
writeDigitalPin(a, 'D9', 0);

% Reverse
% writeDigitalPin(a, 'D6', 0);
% writeDigitalPin(a, 'D7', 1);
% writeDigitalPin(a, 'D8', 0);
% writeDigitalPin(a, 'D9', 1);

resetCount(encoder);

writePWMDutyCycle(a,'D5',1.0);

t0 = clock;
while(etime(clock, t0) < 11.15)
   count = readCount(encoder);
   fprintf('Current knob position: %d\n',count)
   pause(0.0001);
end

writePWMDutyCycle(a,'D5',0.0);

cycles = 9;
pulses_per_revolution = abs(count) / cycles;

%% Measure speed
clear encoder
ppr = double(uint32(pulses_per_revolution / 4)); % for quadrature
encoder = rotaryEncoder(a,'D2','D3', ppr);

writePWMDutyCycle(a,'D5',0.5);

t0 = clock;
resetCount(encoder);
while(etime(clock, t0) < 11.15)
    rpm = readSpeed(encoder);
    fprintf('Current rps: %f\n',rpm/60)
    pause(0.01);
end
writePWMDutyCycle(a,'D5',0.0);

%% Create speed per pwm graph

pprs = 0:0.05:1;
count = 1:length(pprs);
rpm = zeros(1, length(pprs));
test_duration = 5;

for i = count
    fprintf('Current pwm tested: %f\n',pprs(i));
    writePWMDutyCycle(a,'D5',pprs(i));
    t0 = clock;
    resetCount(encoder);
    while(etime(clock, t0) < test_duration)
        rpm(i) = readSpeed(encoder);
        pause(0.01);
    end
    writePWMDutyCycle(a,'D5',0.0);
end

plot(pprs, -rpm/60)
xlabel('PWM cycle')
ylabel('Rotation Speed RPS')
grid minor
title('Speed of PWM vs Rotational Speed')