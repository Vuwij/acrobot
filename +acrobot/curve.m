classdef curve < handle
    %CURVE Summary of this class goes here
    %   Parameters for a single curve
    
    properties
        
        % Curve Parameters (Can vary with alternating foot)
        beta = pi/5.8;              % Angle which to hit the ground
        impact_angle = 0.0;         % Angle to impact the ground
        impact_velocity = 0.0;      % In terms of q1 q2 norm
        energy_loss = 1.0;
        pre_impact_angle = -pi + pi/2; % Angle of preimpact before stop torque
        pre_impact_torque = 0;

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
        function obj = curve(beta, impact_angle, impact_velocity, energy_loss, pre_impact_angle, pre_impact_torque)
            obj.beta = beta;
            obj.impact_angle = impact_angle;
            obj.impact_velocity = impact_velocity;
            obj.energy_loss = energy_loss;
            obj.pre_impact_angle = pre_impact_angle;
            obj.pre_impact_torque = pre_impact_torque;
        end
    end
end

