classdef curve < handle
    %CURVE Summary of this class goes here
    %   Parameters for a single curve
    
    properties
        
        % Curve Parameters (Can vary with alternating foot)
        beta = pi/8;                % Angle which to hit the ground
        energy_loss = 1;            % Energy lost from hitting the ground
        impact_velocity = 2.5*pi;   % Velocity to hit the ground
        
        % Computed Curve parameters
        qm; % q1-q2 angle pre-impact
        qp; % q1-q2 angle post-impact
        w;
        v;
        
        f_func;         % No Torque function for falling function
        r_func;         % No function for rising function with constant tau
        
        tau_const = 0;  % Constant tau for it to reach the starting to the end point
    end
    
    methods
        function obj = curve()
            %
        end
    end
end

