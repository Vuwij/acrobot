% Data plotting for ECE557 lab 4.
% Last modified August 29, 2017.

clear all;
close all;
clc;
delete(instrfindall);

% Ensure the following serial port is set to the same serial port being
% used by the Arduino controller.
serialPort = 'COM4';

%% Create the serial object
serialObject = serial(serialPort, 'BaudRate', 115200);
fopen(serialObject);

% Set the time span and interval for data collection
stop_time = 50;
T_plot = 0.03;
n_samples = fix(stop_time/T_plot);


%% Set up the figure window

figure_handle = figure('NumberTitle','off', 'Name','Tracking');

% Set axes
axes_handle = axes('Parent',figure_handle, 'YGrid','on', 'XGrid','on');
hold on

plot_handle = plot(axes_handle, 0,0, 'Marker','.', 'LineWidth',1);
xlim(axes_handle, [0 stop_time]);
ylim(axes_handle, [-0.05 0.05]);

% Create xlabel
xlabel('Time [s]', 'FontWeight','bold', 'FontSize',14);

% Create ylabel
ylabel('Position [m]', 'FontWeight','bold', 'FontSize',14);

%Create title
title('Cart Position as a Function of Time', ...
    'FontWeight','bold', 'FontSize',15);


%% Collect data
time = T_plot*(0:n_samples-1);
zero_matrix=zeros(1, n_samples);
cart_position_encoder=zero_matrix;
pendulum_position_encoder=zero_matrix;
cart_position_ref=zero_matrix;
cart_speed_obs=zero_matrix;
pendulum_speed_obs=zero_matrix;

for count=1:n_samples
    cart_position_encoder(count) = fscanf(serialObject,'%f') ; 
    pendulum_position_encoder(count) = fscanf(serialObject,'%f') ;
    cart_position_ref(count) = fscanf(serialObject,'%f');
    cart_speed_obs(count)=fscanf(serialObject, '%f');
    pendulum_speed_obs(count)=fscanf(serialObject,'%f');
    
    % update the plot
      set(plot_handle, 'YData', cart_position_encoder(1:count), 'XData',time(1:count));
      set(figure_handle, 'Visible','on');
end

%% Compute the velocity and filtered velocity

% Obtain cart velocity from cart position encoder;
    position_pp = spline(time, cart_position_encoder);
    velocity_pp = fnder(position_pp);
    cart_speed_encoder = ppval(velocity_pp, time);
    % Smooth velocity data using an aggressive Butterworth LP filter
       [b,a] = butter(10, 0.25);
       cart_speed_encoder = filtfilt(b, a, cart_speed_encoder);

% Obtain pendulum velocity from pendulum position encoder;
    pendulum_pp = spline(time, pendulum_position_encoder);
    pendulum_velocity_pp = fnder(pendulum_pp);
    pendulum_speed_encoder = ppval(pendulum_velocity_pp, time);
    % Smooth velocity data using an aggressive Butterworth LP filter
       [b,a] = butter(5, 0.25);
       pendulum_speed_encoder = filtfilt(b, a, pendulum_speed_encoder);
      

%% Plot all outstanding data

hline2 = plot(time, cart_position_ref, 'r', 'Linewidth',2);
hline1 = plot(time, cart_position_encoder, 'b', 'Marker','.','Linewidth',1);

xlabel('Time [s]', 'FontWeight','bold', 'FontSize',14)
ylabel('Position [m]', 'FontWeight','bold', 'FontSize',14);
legend([hline1,hline2], 'Position', 'Reference Position');

hold off;

% pendulum angle
figure;
plot(time, pendulum_position_encoder);
xlabel('Time [s]', 'FontWeight','bold', 'FontSize',14);
ylabel('Angle [rad]', 'FontWeight','bold', 'FontSize',14);
title('Pendulum Angle as a Function of Time', ...
    'FontWeight','bold', 'FontSize',15);
grid on;

% cart speed
figure;
plot(time, cart_speed_obs, time, cart_speed_encoder);
xlabel('Time [s]', 'FontWeight','bold', 'FontSize',14);
ylabel('Speed [m/s]', 'FontWeight','bold', 'FontSize',14);
title('Cart Speed as a Function of Time', ...
    'FontWeight','bold', 'FontSize',15);
legend('Cart speed observer (m/s)','Cart speed (m/s)')
grid on;

% pendulum speed
figure;
plot(time, pendulum_speed_obs, time, pendulum_speed_encoder);
xlabel('Time [s]', 'FontWeight','bold', 'FontSize',14);
ylabel('Pendulum Speed [rad/s]', 'FontWeight','bold', 'FontSize',14);
title('Pendulum Angular Speed as a Function of Time', ...
    'FontWeight','bold', 'FontSize',15);
legend('Pendulum speed observer (rad/sec)','Pendulum speed (rad/sec)')
grid on;

%% Clean up the serial object
fclose(serialObject);
delete(serialObject);
clear serialObject;
