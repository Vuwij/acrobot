% Walking Robot Parameters
% Copyright 2017 The MathWorks, Inc.

%% General parameters
density = 1000;
foot_density = 2000;

%% Contact/friction parameters
% Contract
contact_stiffness = 1e4;
contact_damping = 1e2;

% Friction
mu_k = 1.0;
mu_s = 0.3;
mu_vth = 0.001;

% Plane Parameters
height_plane = 0.025;
plane_x = 25;
plane_y = 3;

%% Foot parameters
foot_radius = 0.0075;
line_depth_to_reference_frame = 0.05;


%% Motor properties
steps_per_rotation = 2797;