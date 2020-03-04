classdef acrobot_control < acrobot.acrobot
    
    properties(Access = protected)
        pheel = [0; 0];      % Position of the heel
        
        q_field_plotted = 0;
        tau = [0; 0];
        tau_q = [0; 0];
        tau_g = [0; 0];
        holo_point = [0; 0];
        holo_point_dt = [0; 0];
    end
    properties
        x = zeros(4,1);     % Current x state space
        
        % Controller parameters
        gamma = 0.10;
        poles = [-20 -20];
        L = 0.9;
        K = 0.9;
        
        % Plots
        tau_limit = 0.30;
    end
    
    methods
        function obj = acrobot_control()
            obj = obj@acrobot.acrobot();
        end
        
        function reset(obj)
            % Start on the cycle
            X = obj.getFallingCurve([obj.c1.qm; obj.c1.w], 0.1, -1);
            obj.x = X(end,:)';
            % X = [1.7671; 2.7489; -1.0573; 0.7951];
            X = [pi/2; 0; 0; -0.01];
            obj.x = X;
            obj.tau = [0; 0];
            obj.holo_point = [0; 0];
            obj.holo_point_dt = [0; 0];
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
        
        function [qd, qd_dot] = getQdesired(obj, x)
            q = [x(1); x(2)];
            qdot = [x(3); x(4)];
            
            % Robotics Equation Parameters
            D = obj.calc_D(obj.linertia(1), obj.linertia(2), obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2),q(2));
            C = obj.calc_C(obj.leg_length, obj.lcom(2), obj.lmass(2), q(2), qdot(1), qdot(2));
            P = obj.calc_P(obj.g, obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2), q(1), q(2));
                        
            phi = @(q2) ppval(obj.lcurve.g_func,q2);
            phi_dot = @(q2) ppval(fnder(obj.lcurve.g_func,1),q2);
            
            qd = [phi(q(2)); q(2)];
            qd_dot = [phi_dot(q(2)) * qdot(2); qdot(2)];            
        end
        
        function pwm = getPWM(obj, qd, qd_dot)
            q = [obj.x(1); obj.x(2)];
            qdot = [obj.x(3); obj.x(4)];
            
            e = qd(2) - q(2)
            e_dot = qd_dot(2) - qdot(2)
            pwm = -obj.Kp * e + obj.Kd * e_dot;
            
            pwm = min(1,max(-1,pwm));
        end
        
        function [tau, qd] = getTau1(obj, x)
            q = [x(1); x(2)];
            qdot = [x(3); x(4)];
            
            % Robotics Equation Parameters
            D = obj.calc_D(obj.linertia(1), obj.linertia(2), obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2),q(2));
            C = obj.calc_C(obj.leg_length, obj.lcom(2), obj.lmass(2), q(2), qdot(1), qdot(2));
            P = obj.calc_P(obj.g, obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2), q(1), q(2));
            
            qddot = D \ (-C * qdot - P); 
            
            phi = @(q2) ppval(obj.lcurve.g_func,q2);
            phi_dot = @(q2) ppval(fnder(obj.lcurve.g_func,1),q2);
            phi_ddot = @(q2) ppval(fnder(obj.lcurve.g_func,2),q2);
            
            qd = [phi(q(2)); q(2)];
            qd_dot = [phi_dot(q(2)) * qdot(2); qdot(2)];
            qd_ddot = [phi_dot(q(2)) * qddot(2) + phi_ddot(q(2)) * qdot(2)^2; qddot(2)];

            
            % PD Control
            e = qd(1) - q(1);
            e_dot = qd_dot(1) - qdot(1);
            
            part1 = [1 -phi_dot(q(2))] * inv(D) * obj.B;
            part2 = -obj.Kp * e - obj.Kd * e_dot + qd_ddot(1) * qdot(2)^2 + [1 -phi_dot(q(2))] * inv(D) * (C * qdot + P);
            obj.tau = [0; inv(part1) * part2];

            obj.tau = max(min(obj.tau_limit, obj.tau), -obj.tau_limit);
            tau = obj.tau;
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
            xfdx = @(q1,q2) fnval( spmak({obj.lcurve.knotsx,obj.lcurve.knotsy},obj.lcurve.sp2_dx.coefs.'), {q1,q2} );
            xfdy = @(q1,q2) fnval( spmak({obj.lcurve.knotsx,obj.lcurve.knotsy},obj.lcurve.sp2_dy.coefs.'), {q1,q2} );
            
            gf = @(q1, q2) fnval(obj.lcurve.sp3, xf(q1, q2));
            gfd = @(q1, q2, q1_dot, q2_dot) fnval(sp3d, xf(q1, q2)) * (xfdx(q1, q2) * q1_dot + xfdy(q1, q2) * q2_dot);
%             gfdd = @(q1, q2) fnval(sp3dd, xfdd(q1, q2));
            
%             xv = 0:pi/100:pi; yv = -pi:2*pi/100:pi;
%             
%             UU = zeros(length(xv), length(yv));
%             DD = zeros(length(xv), length(yv));
%             for i = 1:length(xv)
%                 for j = 1:length(yv)
%                     UU(i,j) = xf(xv(i), yv(j));
%                     DD(i,j) = xfd(xv(i), yv(j));
%                 end
%             end
%             figure;
%             mesh(xv, yv, UU)
%             figure;
%             mesh(xv, yv, DD)
            qd = gf(q(1),q(2));
            qd_dot = gfd(q(1), q(2), qdot(1),qdot(2));
            qdot
            qd_dot
%             qd_ddot = gfd(qddot(1),qddot(2)) + gfdd(qdot(1)^2, qdot(2)^2);
                        
            obj.holo_point = qd;
            obj.holo_point_dt = qd_dot;
            
            % PD Control
            dist = qd - q;
            ang1 = atan2(dist(2), dist(1));
            ang2 = atan2(qd_dot(2), qd_dot(1));
            qs = abs(dist) * sign(ang1 - ang2);
            
            qs_dot = qd_dot - qdot;
%             qs_ddot = qd_ddot - qddot;

%            a = (obj.Kp * qs(2) + obj.Kd * qs_dot(2) + qs_ddot(2)); % Acceleration of q2
            a = (obj.Kp * qs(2));% + obj.Kd * qs_dot(2));
            
            obj.tau = [0; inv([-qd_dot(2) 1] * inv(D) * obj.B) * a];
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
        
        function plotRobot(obj)
            subplot(2,3,1);
            
            q1 = obj.x(1);
            q2 = obj.x(2);
            
            rc1 = (obj.leg_length - obj.lcom(1)) * [cos(q1); sin(q1)] + obj.pheel;
            rH = obj.leg_length * [cos(q1); sin(q1)] + obj.pheel;
            rc2 = rH + obj.lcom(2) * [cos(q1+q2); sin(q1+q2)];
            pH2 = rH + obj.leg_length * [cos(q1+q2); sin(q1+q2)];
            
            hold off;
            plot(rc1(1), rc1(2), '.', 'markersize',20,'color','b');     % Stance leg mass
            hold on;
            plot(rc2(1), rc2(2), '.', 'markersize',20,'color','b');     % Swing leg mass
            r1 = line([obj.pheel(1);rH(1)],[obj.pheel(2),rH(2)]);       % heel1 to hip
            r2 = line([rH(1),pH2(1)],[rH(2),pH2(2)] );                  % hip to heel2

            r1.Color = 'blue';
            r2.Color = 'black';
            
            grid on;
            axis equal;
            ylim([-0.5, 1]);
            xlim([-0.5, 2]);
        end
        
        function plotFields(obj)
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
            
            q1 = obj.x(1);
            q2 = obj.x(2);
            
            plot(q1(1), q2(1), '.', 'markersize',10,'color',[0 0 0]);
            plot(obj.holo_point(1), obj.holo_point(2), '.', 'markersize',10,'color',[0 1 0]);
            quiver(obj.holo_point(1), obj.holo_point(2), obj.holo_point_dt(1) * 0.5, obj.holo_point_dt(2) * 0.5);

            quiver(q1(1), q2(1), obj.tau_q(1) * 0.0005, obj.tau_q(2) * 0.0005);
        end
        
        function plotTau(obj, t)
            subplot(2,3,4);
            hold on;
            plot(t, obj.tau(2), '+', 'markersize',5,'color','m');
            ylabel('Tau N*m');
            xlabel('Time (s)');
            ylim([-obj.tau_limit - 0.1 obj.tau_limit + 0.1]);
            grid on;
        end
        
        function show(obj, t)
            % Plot Robot
            obj.plotRobot();
            
            % Plot q1,q2 field
            obj.plotFields();

            % Plotting tau
            obj.plotTau(t);

            drawnow;
        end
    end
end
