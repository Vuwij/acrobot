classdef curve < handle
    %CURVE Summary of this class goes here
    %   Parameters for a single curve
    
    properties
        
        % Curve Parameters (Can vary with alternating foot)
        beta = pi/5.8;             % Angle which to hit the ground
        impact_velocity = 21.0;    % In terms of q1 q2 norm
        
        % Computed Curve parameters
        qm; % q1-q2 angle pre-impact
        qp; % q1-q2 angle post-impact
        w;  % Preimpact vector
        v;  % Postimpact vector
        
        f_func;         % No Torque function for falling function
        r_func;         % Function for rising function with constant tau
        g_func;         % Function g_func(q2) = q1
        v_func;         % Function of velocity desired as compared to 
        
        tau_const = 0;  % Constant tau for it to reach the starting to the end point
        
        % Holonomic function parameters
        kx = 3;
        ky = 4;
        knotsx;
        knotsy;
        
        sp2;    % Function from q1, q2 to x axis
        sp2_dx;
        sp2_dy;

        sp3; % Function from x axis to the r_func
    end
    
    methods
        function obj = curve()
            % Do nothing
        end
    end
end

