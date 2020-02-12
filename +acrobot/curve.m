classdef curve < handle
    %CURVE Summary of this class goes here
    %   Parameters for a single curve
    
    properties
        % VHC parameters
        BX;
        BY;
        BU;
        BV;
        
        % Output curves properties
        delta_qsdot;
        sigma;
        theta_f;
        theta_start;
        theta_end;
        
        % Curve parameters
        qm;
        qp;
        w;
        v;
        
        g_func;
    end
    
    methods
        function obj = curve()
            %
        end
        
        function plotQField(obj)
            q1_range = 0:0.05:pi;
            
%             quiver(obj.BX,obj.BY,obj.BU,obj.BV) %orbits
            streamslice(obj.BX,obj.BY,obj.BU,obj.BV,'color','cyan') %orbits
            
            hold on;

            % Plot impact surfaces S+, S-
            plot(q1_range, -2 * q1_range + 2 * pi,'color','cyan');
            plot(q1_range, -2 * q1_range, 'color','cyan');
            xlim([0, pi]);
            ylim([-pi, pi]);
            
            hold off;
        end
    end
end

