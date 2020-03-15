%% General parameters
density = 1000;
foot_density = 2000;
g = 9.81;

%% Contact/friction parameters
% Contract
contact_stiffness = 1e3;
contact_damping = 1e2;

% Friction
mu_k = 1.0;
mu_s = 0.3;
mu_vth = 0.001;

% Plane Parameters
height_plane = 0.025;
plane_x = 1;
plane_y = 0.5;

%% Foot parameters
leg_length = 0.3480;
foot_radius = 0.018;
line_depth_to_reference_frame = 0.05;


%% Motor properties
steps_per_rotation = 2797;

%% IMU parameters
imu_sample_rate = 100;
gyro_noise = 0.001;
acc_noise = 0.01;