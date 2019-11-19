classdef acrobot_anim < acrobot.acrobot
    
    properties
        x = zeros(4,1); % Current x state space
        gamma = 0.05;
        g = 9.81;
        pheel = [0; 0];      % Position of the heel
        
        
        % Plots
        show_plot = 0;
        q_field_plotted = 0;
        tau = 0;
        e = 0;
        
        tau_limit = 50;
    end
    
    methods
        function obj = acrobot_anim()
            obj = obj@acrobot.acrobot();
        
            %Initial conditions (Already moving)
%             q = ppval(fnder(obj.sigma,1),0) * 7.2295;                               % Values when on the limit cycle
%             obj.x = [(pi + obj.beta)/2 - 0.05; pi - obj.beta; q(1) - 1; q(2) + 1];  % Currently set to start off the configuration manifold

            % Initial conditions (standing)
            obj.x = [pi/2; 0; 0; -1];
        end
        
        function Kp = Kp(obj)
            Kp = (1 / obj.gamma) ^ 2;
        end
        
        function Kd = Kd(obj)
            Kd = 2 / obj.gamma;
        end
        
        function [dist, isterminal, direction] = dist_to_floor(obj, t, x)
            q1 = x(1);
            q2 = x(2);
            rH = obj.leg_length * [cos(q1); sin(q1)];
            rc2 = rH + obj.leg_length * [cos(q1+q2); sin(q1+q2)];
            dist = rc2(2);
            
            obj.x = x;

            direction = 0;
            isterminal = 1;
            
            if obj.show_plot
                obj.show(t);
            end
        end
        
        function dxdt = step(obj, ~, x)
            q1 = x(1);
            q2 = x(2);
            q1dot = x(3);
            q2dot = x(4);

            qdot = [q1dot; q2dot];
            D = obj.calc_D(obj.leg_length, obj.com(1), obj.com(2), obj.mass(1), obj.mass(2),q2);
            b = obj.calc_b(obj.g, obj.leg_length, obj.com(1), obj.com(2), obj.mass(1), obj.mass(2),q1,q2,q1dot,q2dot);

            g_prime = fnder(obj.g_func,1);
            g_d_prime = fnder(obj.g_func,2);

            g_val = ppval(obj.g_func,q1);
            g_prime_val = ppval(g_prime,q1);
            g_d_prime_val = ppval(g_d_prime,q1);

            obj.e = q2 - g_val;
            e_dot = q2dot - g_prime_val*q1dot;

            tmp1 = inv([-g_prime_val 1] * inv(D) * obj.B);
            tmp2 = (-obj.Kp * obj.e - obj.Kd * e_dot + g_d_prime_val * q1dot ^ 2 + [-g_prime_val 1] * inv(D) * (-b));
            
            tau_new = tmp1 * tmp2;
            obj.tau = max(min(obj.tau_limit, tau_new), -obj.tau_limit);

            qddot = D \ (obj.B * obj.tau) + D \ b; 

            dxdt = [qdot; qddot];
        end
        
        function impact_map(obj, x)

            q1 = x(1);      % pre-impact
            q2 = x(2);      % pre-impact
            qs = [q1; q2];
            
            q1_dot_m = x(3);
            q2_dot_m = x(4);
            qs_dot= [q1_dot_m; q2_dot_m];

            De = obj.calc_De(obj.leg_length, obj.com(1), obj.com(2), obj.mass(1), obj.mass(2),q1, q2);
            E = obj.calc_E(obj.leg_length, obj.leg_length, q1,q2);

            dUde = obj.calc_dUde(); 
            last_term = [eye(2); dUde];

            delta_F = -((E/De)*transpose(E))\E*last_term;
            delta_qedot = (De\transpose(E))*delta_F + last_term;

            % Relabelling
            T = [1 1; 0 -1];
            delta_qsdot = [T zeros(2,2)] * delta_qedot;

            % Relabelled
            q_p_r = T * qs + [-pi; 0];
            qdot_p_r = delta_qsdot*qs_dot;

            % Update the X term
            obj.x = [q_p_r ; qdot_p_r];
            obj.x(1:2) = wrapTo2Pi(obj.x(1:2));
        end
        
        function update_heel(obj)
            % Pre-impact configuration
            q1 = obj.x(1);
            q2 = obj.x(2);

            rH = obj.leg_length * [cos(q1); sin(q1)];                       % Hip position
            step_diff = rH + obj.leg_length * [cos(q1+q2); sin(q1+q2)];     % Swing foot position
            obj.pheel = obj.pheel + step_diff;
            obj.pheel(2) = 0;
        end
        
        function show(obj, t)

            % Plot Robot
            subplot(2,2,1)
            
            hold off;
            Xslope_plot = linspace(-10,10,100);
            Yslope_plot = 0 * Xslope_plot;
            plot(Xslope_plot,Yslope_plot)

            axis_vec = [-0.6 0.8 -0.5 0.9];
            axis square
            axis(axis_vec);

            % Heel of stance foot
            q1 = obj.x(1);
            q2 = obj.x(2);
            
            rc1 = obj.com(1) * [cos(q1); sin(q1)] + obj.pheel;
            rH = obj.leg_length * [cos(q1); sin(q1)] + obj.pheel;
            rc2 = rH + obj.com(2) * [cos(q1+q2); sin(q1+q2)];
            pH2 = rH + obj.leg_length * [cos(q1+q2); sin(q1+q2)];
            
            % Scroll the view
            if ((pH2(1) + 0.1) > axis_vec(2))
                axis_shift = axis_vec(2) - axis_vec(1) - 0.2;
                axis_vec = axis_vec+[axis_shift axis_shift 0 0];
                axis(axis_vec);
            end
            
            hold on;
            plot(rc1(1), rc1(2), '.', 'markersize',20,'color','b');     % Stance leg mass
            plot(rc2(1), rc2(2), '.', 'markersize',20,'color','b');     % Swing leg mass
            r1 = line([obj.pheel(1);rH(1)],[obj.pheel(2),rH(2)]);   % heel1 to hip
            r2 = line([rH(1),pH2(1)],[rH(2),pH2(2)] );              % hip to heel2

            r1.Color = 'blue';
            r2.Color = 'black';

            % Plotting the subplot field
            subplot(2,2,2);
            if ~obj.q_field_plotted
                obj.plotQField();
                hold on;
                obj.q_field_plotted = 1;
                ylabel('q2');
                xlabel('q1');
            end
            
            plot(q1(1), q2(1), '.', 'markersize',3,'color','m');
            
            % Plotting tau
            subplot(2,2,3);
            hold on;
            plot(t, obj.tau, '.', 'markersize',3,'color','m');
            ylabel('Tau N*m');
            xlabel('Time (s)');
            
            % Plotting error
            subplot(2,2,4);
            hold on;
            plot(t, obj.e, '.', 'markersize',3,'color','m');
            ylabel('Error');
            xlabel('Time (s)');
            
%            text(-0.8,0.5,sprintf('time: %f', t)); % Display current time
            drawnow;
        end
    end
end
