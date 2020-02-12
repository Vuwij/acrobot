classdef acrobot < handle
    
    properties(Access = private)
        % Computed Curve Parameters
        p1;
        p2;
        p3;
        p4;
    end
    properties(Access = protected)
        robot = importrobot("acrobot_description/models/acrobot.urdf");
        
        % Convenience values
        mass = zeros(2,1);
        com = zeros(2,1);
        inertia = zeros(2,1);
        
        % Function Handles
        calc_D;
        calc_C;
        calc_P;
        calc_EE;
        
        calc_J;
        calc_De;
        calc_E;
        calc_dUde;
        
        % VHC Parameters
        B = [0; 1];
        
        % Curves
        c1 = acrobot.curve();
        c2 = acrobot.curve();
    end
    properties
        % Physical Parameters
        g = 9.81;
        
        % Mechanical Parameters
        leg_length = 0.335;
        foot_radius = 0.0075;

        % Curve Parameters
        beta = pi/8;
        vwscale = 1;
        impact_velocity = 2.5*pi; % To be computed using energy considerations

        step_count = 0;
    end
    
    methods
        function obj = acrobot()
            obj.robot.showdetails
            
            % Calculations
            for i = 1:2
                obj.mass(i) = obj.robot.Bodies{i}.Mass;
                obj.com(i) = norm(obj.robot.Bodies{i}.CenterOfMass(1));
                obj.inertia(i) = obj.robot.Bodies{i}.Inertia(2);
            end

            % Create Robot Equation handles
            obj.solveRoboticsEquation();
            
            % Solve for the curve for both legs
            obj.calcHolonomicCurve(obj.c1);
            obj.calculateQField(obj.c1);
            obj.step_count = 1;
            obj.calcHolonomicCurve(obj.c2);
            obj.calculateQField(obj.c2);
            obj.step_count = 0;

            % Calculate Q Curve
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
        
        function inertia = linertia(obj, num)
            if rem(obj.step_count,2) == 1
                if num == 2
                    num = 1;
                else
                    num = 2;
                end
            end
            inertia = obj.inertia(num);
        end
        
        function curve = lcurve(obj)
            if rem(obj.step_count,2) == 1
                curve = obj.c1;
            else
                curve = obj.c2;
            end
        end
        
        function t_value = bezier(~, precision, p1, p2, p3, p4)
            t_range = 0:precision:1;
            t_value = zeros(2,length(t_range));
            
            for s = 1:length(t_range)
                t = t_range(s);
                for d = 1:2
                    bez_param = [p1(d) p2(d) p3(d) p4(d)];

                    % Cubic bezier
                    for i = 0:3
                        t_value(d, s) = t_value(d, s) + bez_param(i+1) * nchoosek(3, i) * (1 - t)^(3 - i) * t ^ i;
                    end
                end
            end
        end
        
        function calcHolonomicCurve(obj, curve)
            
            curve.qm = [(pi - obj.beta)/2; obj.beta - pi]; % Joint angles pre impact
            curve.qp = [(pi + obj.beta)/2; pi - obj.beta]; % Joint angles post impact
            
            
            % Use the gravity potential field for calculation of w
            D = obj.calc_D(obj.linertia(1), obj.linertia(2), obj.leg_length, obj.lcom(1), obj.lcom(2), ...
                            obj.lmass(1), obj.lmass(2), curve.qm(2));
            P = obj.calc_P(obj.g, obj.leg_length, obj.lcom(1), obj.lcom(2), ...
                            obj.lmass(1), obj.lmass(2), curve.qm(1), curve.qm(2));
            J = obj.calc_J(obj.leg_length,obj.leg_length, curve.qm(1), curve.qm(2));

            min_angle = 2*pi;
            for angle = -pi:0.001:0 % Search for the angle of impact with the most natural fall
                wt = [cos(angle); sin(angle)] * obj.impact_velocity;
                qdot = J \ wt;

                C = obj.calc_C(obj.leg_length, obj.lcom(2), obj.lmass(2), curve.qm(2), qdot(1), qdot(2));
                qddt = D \ (-C * qdot - P);
                if abs(angdiff(atan2(qddt(2), qddt(1)),angle)) < min_angle
                    curve.w = qdot;
                    min_angle = abs(angdiff(atan2(qddt(2), qddt(1)),angle));
                end
            end
            
            % Post impact calculations
            De = obj.calc_De(obj.linertia(1), obj.linertia(2), obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2), curve.qm(1), curve.qm(2));
            E = obj.calc_E(obj.leg_length, obj.leg_length, curve.qm(1), curve.qm(2));
            dUde = obj.calc_dUde(obj.leg_length, curve.qm(1));
            last_term = [eye(2); dUde];

            delta_F = -(E/De*E')\E*last_term;
            delta_qedot = De\E'*delta_F + last_term;
            T = [1 1; 0 -1]; % Relabelling
            qp_dot = [T zeros(2,2)] * (delta_qedot * curve.w);
            rend_dot = obj.calc_J(obj.leg_length, obj.leg_length, curve.qp(1), curve.qp(2)) * qp_dot;
            if (rend_dot(2) < 0)
                curve.v = -qp_dot;
            else
                curve.v = qp_dot;
            end
            
            % Use post fall curve
            X = obj.getFallingCurve([curve.qm; curve.w], 0.3, -1);
            curve.g_func = cscvn(X(:,1:2)');
        end
        
        function plotHolonomicCurve(obj, curve)
            curve.plotQField();
            hold on;
            obj.plotPField();
            % obj.plotControllerSensitivityField();
            obj.plotFallingCurve(curve);

            plot(curve.qp(1), curve.qp(2),'o', 'MarkerSize',5,'color','k');
            plot(curve.qm(1), curve.qm(2),'o', 'MarkerSize',5,'color','k');
            
            fnplt(curve.g_func);
            
            title('Plot of sigma(theta)')
            xlabel('q1')
            ylabel('q2')
            axis equal
            xlim([0, pi])
            hold off;
        end
        
        function calculateQField(obj, curve)
            % Plot vector field inv(D)*B
            q1_range = 0:0.05:pi;
            q2_range = -pi:0.05:pi;
            [curve.BX,curve.BY] = meshgrid(q1_range,q2_range);
            
            curve.BU = zeros(size(curve.BX));
            curve.BV = zeros(size(curve.BY));

            % Dinv * B
            m1 = obj.lmass(1);
            m2 = obj.lmass(2);
            for i=1:size(curve.BX,1)
                for j=1:size(curve.BY,2)
                    temp = obj.calc_D(obj.linertia(1), obj.linertia(2), obj.leg_length, obj.lcom(1), obj.lcom(2), ...
                                    m1, m2, curve.BY(i,j)) \ obj.B;
                    curve.BU(i,j) = temp(1);
                    curve.BV(i,j) = temp(2);
                end
            end            
        end
        
        function plotPField(obj)
            % Plot vector field inv(D)*B
            finity = 0.2;
            q1_range = 0:finity:pi;
            q2_range = -pi:2*finity:pi;
            [X1,X2] = meshgrid(q1_range,q2_range);
            
            R = zeros(size(X1));
            Z = zeros(size(X2));

            % Dinv * B
            m1 = obj.lmass(1);
            m2 = obj.lmass(2);
            for i=1:size(X2,1)
                for j=1:size(X2,2)
                    D = obj.calc_D(obj.linertia(1), obj.linertia(1), obj.leg_length, obj.lcom(1), obj.lcom(2), ...
                                    m1, m2, X2(i,j));
                    P = obj.calc_P(obj.g, obj.leg_length, obj.lcom(1), obj.lcom(2), ...
                                    m1, m2, X1(i,j), X2(i,j));
                    temp = D \ -P;
                    R(i,j) = temp(1);
                    Z(i,j) = temp(2);
                end
            end
            
            quiver(X1,X2,R,Z,'color',[1 0 1])
        end

        function X = getFallingCurve(obj, X_s, tend, dir)
            % Basically perform a fall in reverse, TODO, use ODE
            tstep = 0.005;
            s = 1;

            % Prefalling phase
            X = zeros(length(0:tstep:tend),4);
            X(1,:) = X_s; % End state
            for t = tstep:tstep:tend
                D = obj.calc_D(obj.inertia(1), obj.inertia(2), obj.leg_length, obj.com(1), obj.com(2), ...
                    obj.mass(1), obj.mass(2), X(s,2));
                P = obj.calc_P(obj.g, obj.leg_length, obj.com(1), obj.com(2), ...
                                obj.mass(1), obj.mass(2), X(s,1), X(s,2));
                C = obj.calc_C(obj.leg_length, obj.com(2), obj.mass(2), X(s,2), X(s,3), X(s,4));
                qddot_new = D \ (-C * X(s, 3:4)' - P);
                dxdt = [X(s, 3:4)'; qddot_new];
                s = s + 1;
                X(s,:) = X(s-1,:) + (dir * dxdt * tstep)';
            end
        end

        function plotFallingCurve(obj, curve)
            % Basically perform a fall in reverse, TODO, use ODE
            X = obj.getFallingCurve([curve.qm; curve.w], 0.3, -1);
            quiver(X(:,1),X(:,2),X(:,3),X(:,4),'color',[0 1 0])
            xlabel('q1');
            ylabel('q2');

            hold on;

            X = obj.getFallingCurve([curve.qp; curve.v], 0.3, 1);
            quiver(X(:,1),X(:,2),X(:,3),X(:,4),'color',[1 1 0])
            xlabel('q1');
            ylabel('q2');
        end

        function plotControllerSensitivityField(obj)
            % Plot vector field inv(D)*B
            finity = 0.02;
            q1_range = 0:finity:pi;
            q2_range = -pi:2*finity:pi;
            [X1,X2] = meshgrid(q1_range,q2_range);

            R = zeros(size(X1));

            % Dinv * B
            m1 = obj.lmass(1);
            m2 = obj.lmass(2);
            obj.calc_P
            for i=1:size(X2,1)
                for j=1:size(X2,2)
                    D = obj.calc_D(obj.leg_length, obj.lcom(1), obj.lcom(2), ...
                                    m1, m2, X2(i,j));
                    P = obj.calc_P(obj.g, obj.leg_length, obj.lcom(1), obj.lcom(2), ...
                                    m1, m2, X1(i,j), X2(i,j));

                    temp1 = D \ obj.B;
                    temp2 = D \ -P;

                    t2_hat = temp2 / norm(temp2);
                    a1 = dot(temp1, t2_hat) * t2_hat;
                    R(i,j) = norm(temp1 - a1);
                end
            end

            surf(X1, X2, R, 'EdgeColor', 'none');

            hold on;
            plot(q1_range, -2 * q1_range + 2 * pi,'color','cyan');
            plot(q1_range, -2 * q1_range, 'color','cyan');
            xlim([0, pi]);
            ylim([-pi, pi]);
            title('Controller Sensitivity')
            xlabel('q1')
            ylabel('q2')
            alpha 0.3
            view(0,90)
        end
        
        function solveRoboticsEquation(obj)
            % Equations of Motion
            
            syms q1 q2 q1dot q2dot q1ddot q2ddot real
            syms l1 l2 lc1 lc2 i1 i2 real
            syms m1 m2 g real

            q = [q1; q2];
            qdot = [q1dot; q2dot];
            qddot = [q1ddot; q2ddot];

            % Positions
            rc1 = (l1 - lc1) * [cos(q1); sin(q1)];
            rH = l1 * [cos(q1); sin(q1)];
            rc2 = rH + lc2 * [cos(q1+q2); sin(q1+q2)];
            rend = rH + l2 * [cos(q1+q2); sin(q1+q2)];
            
            % Velocities
            rc1dot = simplify(jacobian(rc1,q) * qdot);
            rc2dot = simplify(jacobian(rc2,q) * qdot);
            renddot = simplify(jacobian(rend,q));
            
            w01 = q1dot;
            w02 = (q1dot+q2dot);
            
            % Kinetic and Potential Energy
            T1 = 0.5 * m1 * (rc1dot' * rc1dot) + 0.5 * i1 * w01^2;
            T2 = 0.5 * m2 * (rc2dot' * rc2dot) + 0.5 * i2 * w02^2;
            T = T1 + T2;
            U1 = m1 * g * rc1(2);
            U2 = m2 * g * rc2(2);
            U = U1 + U2;
            L = simplify(T - U);
            EE = T + U;
            dLdq = jacobian(L,q)';
            dLdqdot = jacobian(L,qdot)';
            ddtdLdqdot = jacobian(dLdqdot,q) * qdot + jacobian(dLdqdot,qdot) * qddot;
            Tau = simplify(ddtdLdqdot - dLdq);

            % Find the C, D, P Matrix
            syms C D P real
            [D, b] = equationsToMatrix(Tau, qddot);
            P = -subs(b, [q1dot, q2dot], [0, 0]);
            C = sym(zeros(length(q)));
            for k = 1:size(q)
                for j = 1:size(q)
                    for i = 1:size(q)
                        Qijk = 0.5*(diff(D(k,j),q(i)) + diff(D(k,i),q(j)) - diff(D(i,j),q(k)));
                        C(k,j) = C(k,j) + simplify(Qijk) * qdot(i);
                    end
                end
            end

            % Impact Map
            % Solving for EOM and impact map following
            % "Feedback Control of Dynamic Bipedal Robot Locomotion" by Grizzle, p. 55
            % Based on the 3 link model on p. 67 and the paper "Asymptotically Stable
            % Walking for Biped Robots: Analysis via Systems with Impulse Effects"

            syms q3 q4 q3dot q4dot q3ddot q4ddot real

            qe = [q1; q2; q3; q4];
            qedot = [q1dot; q2dot; q3dot; q4dot];
            qeddot = [q1ddot; q2ddot; q3ddot; q4ddot];

            % Positions
            e = [q3; q4];
            rc1e = rc1 + e;
            rHe = rH + e;
            rc2e = rc2 + e;
            rende = rend + e;

            rc1edot = simplify(jacobian(rc1e, qe) * qedot);
            rc2edot = simplify(jacobian(rc2e, qe) * qedot);

            % Kinetic and Potential Energy
            Te1 = 0.5 * m1 * (rc1edot' * rc1edot) + 0.5 * i1 * w01^2;
            Te2 = 0.5 * m2 * (rc2edot' * rc2edot) + 0.5 * i2 * w02^2;
            Te = Te1 + Te2 ;
            Ue1 = m1 * g * rc1e(2);
            Ue2 = m2 * g * rc2e(2);
            Ue = Ue1 + Ue2;
            Le = simplify(Te - Ue);

            % Finding the EOM
            dLedq = jacobian(Le,qe)';
            dLedqdot = jacobian(Le,qedot)';
            ddtdLedqdot = simplify(jacobian(dLedqdot, qe) * qedot + jacobian(dLedqdot, qedot) * qeddot);
            Taue = simplify(ddtdLedqdot - dLedq);

            % Solve for D matrix
            [De, ~] = equationsToMatrix(Taue, qeddot);

            % Upsilons
            E = simplify(jacobian(rende, qe));
            dUde = simplify(jacobian(rHe,q));
            
            % Add the matlab functions
            obj.calc_J = matlabFunction(renddot);
            obj.calc_D = matlabFunction(D);
            obj.calc_C = matlabFunction(C);
            obj.calc_P = matlabFunction(P);
            obj.calc_De = matlabFunction(De);
            obj.calc_E = matlabFunction(E);
            obj.calc_dUde = matlabFunction(dUde);
            obj.calc_EE = matlabFunction(EE);
        end
    end
end
