classdef curve < handle
    %CURVE Summary of this class goes here
    %   Parameters for a single curve
    
    properties
        
        % Curve Parameters (Can vary with alternating foot)
        beta = pi/5.8;             % Angle which to hit the ground
        impact_velocity = 0.0;    % In terms of q1 q2 norm
        w_ang_diff = 0.0;
        energy_loss = 1.0;
        
        % Computed Curve parameters
        qm; % q1-q2 angle pre-impact
        qp; % q1-q2 angle post-impact
        w;  % Preimpact vector
        v;  % Postimpact vector

        phi;         % Function phi(q2) = q1
        phi_dot;
        phi_ddot;
        
        tau_const = 0;  % Constant tau for it to reach the starting to the end point
        
    end
    
    methods
        function obj = curve(beta, impact_velocity, w_ang_diff, energy_loss)
            obj.beta = beta;
            obj.impact_velocity = impact_velocity;
            obj.w_ang_diff = w_ang_diff;
            obj.energy_loss = energy_loss;
        end
    end
end

