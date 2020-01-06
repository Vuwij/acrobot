classdef acrobot_control < acrobot.acrobot
    
    properties(Access = protected)
        pheel = [0; 0];      % Position of the heel
        
        q_field_plotted = 0;
        tau = 0;
        e = 0;
    end
    properties
        g = 9.81;
        x = zeros(4,1);     % Current x state space
        
        % Controller parameters
        gamma = 0.05;
        poles = [-20 -20];
        L = 0.9;
        K = 0.9;
        
        % Plots
        tau_limit = 10;
        step_count = 0;
    end
    
    methods
        function obj = acrobot_control()
            obj = obj@acrobot.acrobot();
        
            % Start on the limit cycle
%             q1 = pi/2;
%             q1dot = -pi;
%             q2 = ppval(obj.g_func, q1);
%             q2dot = -2*pi; %ppval(fnder(obj.g_func,1),q1) * q1dot;
%             obj.x = [q1; q2; q1dot; q2dot];
            
            % Original Starting point
%             q_dot_0 = ppval(fnder(obj.sigma,1),0)*7.2295; %Values when on the limit cycle
%             x0 = [(pi+obj.beta)/2 - 0.05; pi-obj.beta; q_dot_0(1)-1; q_dot_0(2)+1];  %Currently set to start off the configuration manifold
%             obj.x = x0;
            
            % Straight up
           obj.x = [pi/2; 0; -pi/10; 0];
        end
        
        function mass = lmass(obj, num)
            if rem(obj.step_count,2) == 1
                if num == 2
                    num = 1;
                else
                    num = 2;
                end
            end
            mass = obj.mass(num);
        end
        
        function com = lcom(obj, num)
            if rem(obj.step_count,2) == 1
                if num == 2
                    num = 1;
                else
                    num = 2;
                end
            end
            com = obj.com(num);
        end
        
        function Kp = Kp(obj)
            Kp = (1/obj.gamma)^2;
%            Kp = -obj.poles(1) * -obj.poles(2);
        end
        
        function Kd = Kd(obj)
            Kd = 2/obj.gamma;
%            Kd = -obj.poles(1) + -obj.poles(2);
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
        end
        
        function dxdt = autostep(obj, t, x)
            dxdt = obj.step(t, x);
        end
        
        function dxdt = step(obj, ~, x)
            obj.tau = obj.getTau(x);
            
            q = [x(1); x(2)];
            qdot = [x(3); x(4)];
            
            % Robotics Equation Parameters
            D = obj.calc_D(obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2),q(2));
            C = obj.calc_C(obj.leg_length, obj.lcom(2), obj.lmass(2), q(2), qdot(1), qdot(2));
            P = obj.calc_P(obj.g, obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2), q(1), q(2));
            qddot_new = D \ (-C * qdot - P) + D \ obj.tau; 
            dxdt = [qdot; qddot_new];
        end
        
        function tau = getTau(obj, x)
            q = [x(1); x(2)];
            qdot = [x(3); x(4)];
            
            % Robotics Equation Parameters
            D = obj.calc_D(obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2),q(2));
            C = obj.calc_C(obj.leg_length, obj.lcom(2), obj.lmass(2), q(2), qdot(1), qdot(2));
            P = obj.calc_P(obj.g, obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2), q(1), q(2));
            
            qddot = D \ (-C * qdot - P); 
            
            % Desired angles
            gf = @(q1) ppval(obj.g_func,q1);
            gfd = @(q1) ppval(fnder(obj.g_func,1),q1);
            gfdd = @(q1) ppval(fnder(obj.g_func,2),q1);
            
            qd = [q(1); gf(q(1))];
            qd_dot = [qdot(1); gfd(q(1)) * qdot(1)];
            qd_ddot = [qddot(1); gfd(q(1)) * qddot(1) + gfdd(q(1)) * qdot(1)^2];
            
            % PD Control
            qs = qd - q;
            qs_dot = qd_dot - qdot;
            qs_ddot = qd_ddot - qddot;
            
            a = (obj.Kp * qs(2) + obj.Kd * qs_dot(2) + qs_ddot(2)); % Acceleration of q2
            obj.tau = [0; inv([-gfd(q(1)) 1] * inv(D) * obj.B) * a];
            obj.tau(1) = 0;
            
            obj.tau = max(min(obj.tau_limit, obj.tau), -obj.tau_limit);
            tau = obj.tau;
        end
        
        function impact_foot(obj, x)
            q1 = x(1);
            q2 = x(2);
            q1_dot = x(3);
            q2_dot = x(4);
            
            q = [q1; q2];
            q_dot = [q1_dot; q2_dot];

            De = obj.calc_De(obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2), q1, q2);
            E = obj.calc_E(obj.leg_length, obj.leg_length, q1, q2);
            dUde = obj.calc_dUde(); 
            last_term = [eye(2); dUde];

            delta_F = -(E/De*E')\E*last_term;
            delta_qedot = De\E'*delta_F + last_term;
            T = [1 1; 0 -1]; % Relabelling
            
            qp = T * q + [-pi; 0];
            qp_dot = [T zeros(2,2)] * (delta_qedot * q_dot);
            
            % TODO: Fix impact map instead of conserve energy
%             qp_dot = [q2_dot; q1_dot];
            
            % Update the X term
            obj.x = [qp; -qp_dot];
            obj.x(1:2) = wrapTo2Pi(obj.x(1:2));
            

            % Change heel location
            rH = obj.leg_length * [cos(q1); sin(q1)];                       % Hip position
            step_diff = rH + obj.leg_length * [cos(q1+q2); sin(q1+q2)];     % Swing foot position
            obj.pheel(1) = obj.pheel(1) + step_diff(1);
            
            % Recalculate the holonomic cruve
            obj.calcHolonomicCurve();
            obj.q_field_plotted = 0;
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
            
            rc1 = (obj.leg_length - obj.lcom(1)) * [cos(q1); sin(q1)] + obj.pheel;
            rH = obj.leg_length * [cos(q1); sin(q1)] + obj.pheel;
            rc2 = rH + obj.lcom(2) * [cos(q1+q2); sin(q1+q2)];
            pH2 = rH + obj.leg_length * [cos(q1+q2); sin(q1+q2)];
            
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
                plotHolonomicCurve(obj);
                obj.q_field_plotted = 1;
                hold on;
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
