clear; clc;
close all;
a = arduino('COM5','Nano3','Libraries',{'RotaryEncoder', 'I2C'});
encoder = rotaryEncoder(a,'D2','D3');

desiredAngle = 15;
measuredAngle = 0;

cpr = 2797; % encoder resolution

% PID parameters
kp = 0.0015;
ki = 0;
kd = 0.0003;

t0 = clock;
lastTime = t0;
lastErr = 1;
errSum = 0;
angles = [];
torques = [];
pwms = [];
time = [];

acrobot = acrobot.acrobot_torque_control();

rate = rateControl(10);

% while(etime(clock, t0) < 50.0)
while(1)    
%     pause(0.00001);

    now = clock;
    timeChange = etime(now, lastTime);
  
    count = readCount(encoder);
    measuredAngle = (360 / cpr) * count
    
    angles = [angles, measuredAngle];
    time = [time, etime(now, t0)];
    
    % PID
    error = desiredAngle - measuredAngle;
    errSum = errSum + (error * timeChange);
    dErr = (error - lastErr) / timeChange;
    u = (kp * error) + (ki * errSum) + (kd * dErr);
    torques = [torques, u];
    
%     if (u > 0)
%         writeDigitalPin(a, 'D6', 1);
%         writeDigitalPin(a, 'D7', 0);
%     else
%         writeDigitalPin(a, 'D6', 0);
%         writeDigitalPin(a, 'D7', 1);
%     end
%     
%     pwm = min(abs(u/12), 0.5)
   

    pwm = acrobot.getPWM(u);
    pwms = [pwms, pwm];
    if (pwm > 0)
        writeDigitalPin(a, 'D6', 1);
        writeDigitalPin(a, 'D7', 0);
    else
        writeDigitalPin(a, 'D6', 0);
        writeDigitalPin(a, 'D7', 1);
    end
    
    writePWMDutyCycle(a,'D9',abs(pwm));

    lastErr = error;
    lastTime = now;

    waitfor(rate);
end

writePWMDutyCycle(a,'D9',0.0);
%%
subplot(2,1,1)
plot(time, angles);
title('Measured Angle vs Time');
xlabel('Time (s)');
ylabel('Angle (degrees)');
subplot(2,1,2)
plot(time, torques);
title('Torque vs Time');
xlabel('Time (s)');
ylabel('Torque');

figure
subplot(2,1,1)
plot(time, angles);
title('Measured Angle vs Time');
xlabel('Time (s)');
ylabel('Angle (degrees)');
subplot(2,1,2)
plot(time, pwms);
title('PWM vs Time');
xlabel('Time (s)');
ylabel('Duty Cycle');
