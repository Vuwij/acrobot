clear; clc;
close all;
a = arduino('/dev/ttyUSB0','Nano3','Libraries',{'RotaryEncoder', 'I2C'});
encoder = rotaryEncoder(a,'D2','D3');

robot = acrobot.acrobot_control();
robot_t_control = acrobot.acrobot_torque_control();
%%
gain = 5;
measuredAngle = 0;
error_tolerance = 2;
frequency = 2;
cpr = 2797; % encoder resolution
encoder.resetCount();

t0 = clock;
lastTime = t0;
lastErr = 1;
errSum = 0;
angles = [];
torques = [];
pwms = [];
time = [];
robot.gamma = 15;
robot.d_gain = 1.8;
delta_t = 0.035;
rate = rateControl(1/delta_t);
t = 0;
t_last = 0;
pwm = 0;
tstart = tic;
while(t < 10)
    t = toc(tstart);
    dt = t - t_last;
    t_last = t;
    if (rem((t / frequency), frequency) > frequency/2)
        desiredAngle = gain;
    else
        desiredAngle = -gain;
    end
    

    count = readCount(encoder);
    measuredAngle = (360 / cpr) * count;
    
    angles = [angles, measuredAngle];
    time = [time, t];
    
    % PID
    error = measuredAngle - desiredAngle;
    derror = (error - lastErr) * dt;
    u = (robot.Kp * error) + (robot.Kd * derror);
    
    torques = [torques, u];

    pwm_new = robot_t_control.getPWM(u);
    if (pwm_new < 0 && pwm >= 0)
        writeDigitalPin(a, 'D6', 1);
        writeDigitalPin(a, 'D7', 0);
    elseif (pwm_new > 0 && pwm <= 0)
        writeDigitalPin(a, 'D6', 0);
        writeDigitalPin(a, 'D7', 1);
    end
    pwm = pwm_new;
    pwms = [pwms, pwm];
    writePWMDutyCycle(a,'D9',abs(pwm));

    lastErr = error;
    waitfor(rate);
end

writePWMDutyCycle(a,'D9',0.0);
%%
subplot(3,1,1)
plot(time, angles);
title('Measured Angle vs Time');
xlabel('Time (s)');
ylabel('Angle (degrees)');
grid minor;

subplot(3,1,2)
plot(time, torques);
title('Torque vs Time');
xlabel('Time (s)');
ylabel('Torque');
grid minor;

subplot(3,1,3)
plot(time, pwms);
title('PWM vs Time');
xlabel('Time (s)');
ylabel('Duty Cycle');
grid minor;

