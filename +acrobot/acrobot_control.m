classdef acrobot_control < acrobot.acrobot
    
    properties(Access = protected)
        pheel = [0; 0];      % Position of the heel
        
        holo_point = [0;0];
        q_field_plotted = 0;
        tau = 0;
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
            X = [1.7671; 2.7489; -1.0573; 0.7951];
        end
        
        function Kp = Kp(obj)
            Kp = (1/obj.gamma)^2;
        end
        
        function Kd = Kd(obj)
            Kd = 2/obj.gamma;
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
        
        function tau = getTau(obj, x)
            q = [x(1); x(2)];
            qdot = [x(3); x(4)];
            
            % Robotics Equation Parameters
            D = obj.calc_D(obj.linertia(1), obj.linertia(2), obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2),q(2));
            C = obj.calc_C(obj.leg_length, obj.lcom(2), obj.lmass(2), q(2), qdot(1), qdot(2));
            P = obj.calc_P(obj.g, obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2), q(1), q(2));
            
            qddot = D \ (-C * qdot - P); 
            
            % Desired angles
            sp2d = fnder(obj.lcurve.sp2, 1);
            sp3d = fnder(obj.lcurve.sp3, 1);
            
            xf = @(q1,q2) fnval( spmak({obj.lcurve.knotsx,obj.lcurve.knotsy},obj.lcurve.sp2.coefs.'), {q1,q2} );
            xfd = @(q1,q2) fnval( spmak({obj.lcurve.knotsx,obj.lcurve.knotsy},sp2d.coefs.'), {q1,q2} );

            gf = @(q1, q2) fnval(obj.lcurve.sp3, xf(q1, q2));
            gfd = @(q1, q2) fnval(sp3d, xfd(q1, q2));
            
            
            
            qd = gf(q(1),q(2));
            qd_dot = gf(q(1),q(2));
            
            obj.holo_point = qd;

            
%             gf = @(q1) ppval(obj.lcurve.r_func,q1);
%             gfd = @(q1) ppval(fnder(obj.lcurve.r_func,1),q1);
%             gfdd = @(q1) ppval(fnder(obj.lcurve.r_func,2),q1);
            
%             qd = [q(1); gf(q(1))];
%             qd_dot = [qdot(1); gfd(q(1)) * qdot(1)];
%             qd_ddot = [qddot(1); gfd(q(1)) * qddot(1) + gfdd(q(1)) * qdot(1)^2];
            
            % PD Control
            qs = qd - q;
            qs_dot = qd_dot - qdot;
%            qs_ddot = qd_ddot - qddot;

%             a = (obj.Kp * qs(2) + obj.Kd * qs_dot(2) + qs_ddot(2)); % Acceleration of q2
            a = (obj.Kp * qs(2) + obj.Kd * qs_dot(2));
            
            test = -gfd(q(1), q(2));
            obj.tau = [0; inv([-test(2) 1] * inv(D) * obj.B) * a];
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
            plot(obj.holo_point(1), obj.holo_point(2), '.', 'markersize',10,'color',[0 1 0]);

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
