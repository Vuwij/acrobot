clear;
close all;

% 1st column: max torque
% 2nd col: max speed
% 3rd col: weight
motor_specs_list= [
    1.76, 251, 205;
    0.637, 380, 160;
    0.69, 216, 136;
    0.6, 500, 98;
    1.18, 350, 98;
    0.98, 210, 96;
    0.69, 150, 160
];

torque = motor_specs_list(:,1);
speed = motor_specs_list(:,2);
weight = motor_specs_list(:,3);

% Plot max torque vs max speed
plot(torque, speed, '.r', 'MarkerSize', 20);
title('Max Torque vs Max Speed');
xlabel('Max Torque (Nm)');
ylabel('Max Speed (rpm)');
axis([0, 2, 0, 600]);

% Plot max torque vs weight
figure;
plot(torque, weight, '.r', 'MarkerSize', 20);
title('Max Torque vs Weight');
xlabel('Max Torque (Nm)');
ylabel('Weight (g)');
axis([0, 2, 0, 300]);

% Plot motot curves (torque vs speed)
figure
s = size(torque);
for i=1:s(1)
    txt = ['motor ',num2str(i)];
    plot([0,torque(i)],[speed(i),0],'--', 'DisplayName', txt);
    hold on;
end
title('Motor curves');
xlabel('Torque (Nm)');
ylabel('Speed (rpm)');
legend show;