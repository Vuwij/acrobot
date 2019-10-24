% Walking Robot Parameters
% Copyright 2017 The MathWorks, Inc.

%% General parameters
density = 1000;
foot_density = 2000;
world_damping = 0.25;
world_rot_damping = 0.25;
if ~exist('actuatorType','var')
    actuatorType = 1;
end

%% Contact/friction parameters
contact_stiffness = 2500;
contact_damping = 100;
mu_k = 0.6;
mu_s = 0.8;
mu_vth = 0.1;
height_plane = 0.025;
plane_x = 25;
plane_y = 3;
contact_point_radius = 1e-4;

%% Foot parameters
foot_radius = 0.0075;
foot_length = 0.008;

%% Leg parameters
leg_radius = 0.75;
leg_length = 20;

%% Torso parameters
torso_radius = 0.03;
torso_length = 0.03;
foot_z = 0;
foot_offset = 0;
init_height = foot_z + leg_length + torso_radius/2 + height_plane/2;


%% Joint parameters
joint_damping = 1;
joint_stiffness = 1;
motion_time_constant = 0.01; %0.025;