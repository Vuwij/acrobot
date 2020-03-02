clear; clc;
close all;
a = arduino('COM7','Nano3','Libraries',{'RotaryEncoder', 'I2C'});
encoder = rotaryEncoder(a,'D2','D3');

desiredAngle = 15;
measuredAngle = 0;

cpr = 2797; % encoder resolution

% PID parameters
kp = 1;
ki = 0.009;
kd = 0.05;

t0 = clock;
lastTime = t0;
lastErr = 1;
errSum = 0;
angles = [];
time = [];

while(etime(clock, t0) < 10.15)
    
    pause(0.0001);
    now = clock;
    timeChange = etime(now, lastTime);
  
    count = readCount(encoder);
    measuredAngle = (360 / cpr) * count;
    
    angles = [angles, measuredAngle];
    time = [time, etime(now, t0)];
    
    % PID
    error = desiredAngle - measuredAngle;
    errSum = errSum + (error * timeChange);
    dErr = (error - lastErr) / timeChange;
    u = (kp * error) + (ki * errSum) + (kd * dErr);
    
    if (u < 0)
        writeDigitalPin(a, 'D6', 1);
        writeDigitalPin(a, 'D7', 0);
    else
        writeDigitalPin(a, 'D6', 0);
        writeDigitalPin(a, 'D7', 1);
    end
    
    pwm = min(abs(u/12), 0.6)
    writePWMDutyCycle(a,'D9',abs(pwm));

    lastErr = error;
    lastTime = now;
end

writePWMDutyCycle(a,'D9',0.0);
plot(time, angles);
title('Measured Angle vs Time');
xlabel('Time (s)');
ylabel('Angle (degrees)');
