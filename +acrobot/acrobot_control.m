classdef acrobot_control < acrobot.acrobot
    
    properties(Access = protected)
        pheel = [0; 0];      % Position of the heel
        
        q_field_plotted = 0;
        tau = 0;
        holo_point;
        tau_q;
        tau_g;
        e = 0;
    end
    properties
        x = zeros(4,1);     % Current x state space
        
        % Controller parameters
        gamma = 0.05;
        poles = [-20 -20];
        L = 0.9;
        K = 0.9;
        
        % Plots
        tau_limit = 1;
    end
    
    methods
        function obj = acrobot_control()
            obj = obj@acrobot.acrobot();
        end
        
        function reset(obj)
            % Start on the cycle
            X = obj.getFallingCurve([obj.c1.qm; obj.c1.w], 0.1, -1);
            obj.x = X(end,:)';
%             obj.x = [pi/2;0;0;0];
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
            
            direction = -1;
            isterminal = 1;
        end
        
        function dxdt = step(obj, ~, x, tau)
            q = [x(1); x(2)];
            qdot = [x(3); x(4)];
            
            % Robotics Equation Parameters
            D = obj.calc_D(obj.linertia(1), obj.linertia(1), obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2),q(2));
            C = obj.calc_C(obj.leg_length, obj.lcom(2), obj.lmass(2), q(2), qdot(1), qdot(2));
            P = obj.calc_P(obj.g, obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2), q(1), q(2));

            obj.tau_q = D \ tau;
            obj.tau_g = D \ (-C * qdot - P);
            qddot_new = obj.tau_g + obj.tau_q;
            dxdt = [qdot; qddot_new];
        end
        
        % Return positive if left, negative if right
        function final_dist = getDistToCurve(obj, q)
            xy1 = stream2(obj.lcurve.BX, obj.lcurve.BY, obj.lcurve.BU, obj.lcurve.BV, q(1), q(2));
            xy2 = stream2(obj.lcurve.BX, obj.lcurve.BY, -obj.lcurve.BU, -obj.lcurve.BV, q(1), q(2));
%             plot(xy1{1}(:,1), xy1{1}(:,2));
%             hold on;
%             plot(xy2{1}(:,1), xy2{1}(:,2));
%             fnplt(obj.g_func);
            
            left_pts = xy1{1}';
            right_pts = xy2{1}';
            pts = fnval(obj.lcurve.g_func, obj.lcurve.g_func.breaks);

            % Inefficient 2D loop (will make more efficient)
            min_val = 10000;
            final_dist = 0;
            dist = 0;
            for i = 1:length(left_pts)
                [val, idx] = min(vecnorm(pts - left_pts(:,i),2));
                if (val < min_val)
                    final_dist = dist;
                    obj.holo_point = pts(:,idx);
                    min_val = val;
                end
                if (i ~= 1)
                    dist = dist + norm(left_pts(:,i) - left_pts(:,i-1));
                end
            end
            dist = 0;
            for i = 1:length(right_pts)
                [val, idx] = min(vecnorm(pts - right_pts(:,i),2));
                if (val < min_val)
                    final_dist = dist;
                    obj.holo_point = pts(:,idx);
                    min_val = val;
                end
                if (i ~= 1)
                    dist = dist - norm(right_pts(:,i) - right_pts(:,i-1));
                end
            end           
            
        end
        
        function tau = getTau(obj, x)
            q = [x(1); x(2)];
            qdot = [x(3); x(4)];
            
            % Robotics Equation Parameters
            D = obj.calc_D(obj.linertia(1), obj.linertia(2), obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2),q(2));
            C = obj.calc_C(obj.leg_length, obj.lcom(2), obj.lmass(2), q(2), qdot(1), qdot(2));
            P = obj.calc_P(obj.g, obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2), q(1), q(2));
            
            tau_g = D \ (-C * qdot - P);
            dir = D \ obj.B;
            dir_norm = dir / norm(dir);
            resist = dot(tau_g, dir_norm) / norm(dir);
            
            dist = obj.getDistToCurve(q);
            obj.tau = [0;resist + obj.Kp * dist];
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

            De = obj.calc_De(obj.linertia(1), obj.linertia(2), obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2), q1, q2);
            E = obj.calc_E(obj.leg_length, obj.leg_length, q1, q2);
            dUde = obj.calc_dUde(obj.leg_length, q1);
            last_term = [eye(2); dUde];

            delta_F = -(E/De*E')\E*last_term;
            delta_qedot = De\E'*delta_F + last_term;
            T = [1 1; 0 -1]; % Relabelling
            
            qp = wrapTo2Pi(T * q + [-pi; 0]);
            qp_dot = [T zeros(2,2)] * (delta_qedot * q_dot);
            
            % Collision with floor?
            rend_dot = obj.calc_J(obj.leg_length, obj.leg_length, qp(1), qp(2)) * qp_dot;
            if (rend_dot(2) < 0)
                obj.x = [qp; -qp_dot];
            else
                obj.x = [qp; qp_dot];
            end

            
            % Increase Step Count
            obj.step_count = obj.step_count + 1;
            
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
            subplot(2,3,1)
            
            hold off;
            Xslope_plot = linspace(-10,10,100);
            Yslope_plot = 0 * Xslope_plot;
            plot(Xslope_plot,Yslope_plot)

            axis_vec = [-0.6 0.8 -0.5 0.9];
            axis equal
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
            if ~obj.q_field_plotted
                step_count = obj.step_count;

                subplot(2,3,[2,5]);
                obj.step_count = 0;
                plotHolonomicCurve(obj, obj.c1);
                hold on;
                
                subplot(2,3,[3,6]);
                obj.step_count = 1;
                plotHolonomicCurve(obj, obj.c2);
                hold on;
                
                obj.q_field_plotted = 1;
                obj.step_count = step_count;
            end
            
            if (rem(obj.step_count,2) == 0)
                subplot(2,3,[2,5]);
            else
                subplot(2,3,[3,6]);
            end
            
            plot(q1(1), q2(1), '.', 'markersize',10,'color',[0 0 0]);
            plot(obj.holo_point(1), obj.holo_point(2), 'x', 'markersize',10,'color',[0 0 1]);
            quiver(q1(1), q2(1), obj.tau_q(1) * 0.001, obj.tau_q(2) * 0.001);

            % Plotting tau
            subplot(2,3,4);
            hold on;
            plot(t, obj.tau, '.', 'markersize',3,'color','m');
            ylabel('Tau N*m');
            xlabel('Time (s)');

            drawnow;
        end
    end
end
