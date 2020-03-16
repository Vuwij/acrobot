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
        
        % Forward and Inverse Kinematics
        calc_qd;
        calc_rend;
        
        % VHC Parameters
        B = [0; 1];
        
    end
    properties
        % Physical Parameters
        g = 9.81;
        
        % Mechanical Parameters
        leg_length;
        foot_radius = 0.018;
        angle_limit = pi/20;

        step_count = 0;
        
        % Energy Loss
        fall_duration = 1.8;            % Max Fall duration (not desired)
        tau_limit = 0.35;
        
        % Curves
        top_clip = 10;
        bottom_clip = 1;
        plot_curves = 0;
        c1 = acrobot.curve(pi/9.6, pi*0.25, 1.70, 0.48); % First Step
        c2 = acrobot.curve(pi/9.6, pi*0.25, 1.66, 0.48); % Second Step
    end
    
    methods
        function obj = acrobot()
            obj.robot.showdetails
        
            t = obj.robot.getTransform(obj.robot.homeConfiguration, 'base_link', 'leg1');
            obj.leg_length = t(1,4) + obj.foot_radius;

            % Calculations
            for i = 1:2
                obj.mass(i) = obj.robot.Bodies{i}.Mass;
                obj.com(i) = norm(obj.robot.Bodies{i}.CenterOfMass(1));
                obj.inertia(i) = obj.robot.Bodies{i}.Inertia(2);
            end
            
            % Create Robot Equation handles
            obj.solveRoboticsEquation();
            
            % Solve for the curve for both legs
            obj.calcHolonomicCurves();
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
            if rem(obj.step_count,2) == 0
                curve = obj.c1;
            else
                curve = obj.c2;
            end
        end
        
        function curve = l2curve(obj)
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
        
        function ang_diff = calcHolonomicCurveHelper(obj, qp, qm, v, fall_duration, pre_impact_angle, pre_impact_tau, impact_angle)
            tau = obj.getBestConstTau(qp, qm, v, fall_duration, pre_impact_angle, pre_impact_tau);
            X = obj.getFallingCurve([qp; v], fall_duration, [0; tau], pre_impact_angle, pre_impact_tau);
            
            final_diff = obj.calc_rend(obj.leg_length, obj.leg_length, X(end-1,1), X(end-1,2)) - obj.calc_rend(obj.leg_length, obj.leg_length, X(end,1), X(end,2));

            final_angle = atan2(final_diff(2), final_diff(1));
            if (norm(X(end,1:2) - qm') > 0.01)
                ang_diff = 1e9 + abs(angdiff(impact_angle, final_angle));
            else
                ang_diff = abs(angdiff(impact_angle, final_angle));
            end
        end
        
        function calcHolonomicCurves(obj)
            % Post impact for one foot is pre-impact for next foot
            obj.c1.qm = [(pi - obj.c1.beta)/2; obj.c1.beta - pi]; % Joint angles pre impact
            obj.c1.qp = [(pi + obj.c2.beta)/2; pi - obj.c2.beta]; % Joint angles post impact
            obj.c2.qm = [(pi - obj.c2.beta)/2; obj.c2.beta - pi]; % Joint angles pre impact
            obj.c2.qp = [(pi + obj.c1.beta)/2; pi - obj.c1.beta]; % Joint angles post impact
            
            obj.step_count = 0;
            [v, w] = obj.getImpactVelocities(obj.c1.qm, obj.c2.qp, obj.c1.impact_angle, obj.c1.impact_velocity);
            obj.c1.w = w;
            obj.c2.v = v;
            obj.step_count = 1;
            [v, w] = obj.getImpactVelocities(obj.c2.qm, obj.c1.qp, obj.c2.impact_angle, obj.c2.impact_velocity);
            obj.c2.w = w;
            obj.c1.v = v;
            
            % Search for rising curve with const tau
            options = optimoptions('fmincon','OptimalityTolerance', 1e-2, 'StepTolerance', 1e-3);

            obj.step_count = 0;
            D = @(tau_info) obj.calcHolonomicCurveHelper(obj.c1.qp, obj.c1.qm, obj.c1.v, obj.fall_duration, tau_info(1), tau_info(2), obj.c1.impact_angle);
            tmp = fmincon(D,[obj.c1.pre_impact_angle; obj.c1.pre_impact_torque], [], [], [], [], [-pi/2, 0], [0, obj.tau_limit], [], options);
            obj.c1.pre_impact_angle = tmp(1);
            obj.c1.pre_impact_torque = tmp(2);
            obj.c1.tau_const = obj.getBestConstTau(obj.c1.qp, obj.c1.qm, obj.c1.v, obj.fall_duration, obj.c1.pre_impact_angle, obj.c1.pre_impact_torque);
            X = obj.getFallingCurve([obj.c1.qp; obj.c1.v], obj.fall_duration, [0; obj.c1.tau_const], obj.c1.pre_impact_angle, obj.c1.pre_impact_torque);
            [~,idx] = unique(X(:,2));
            X_ext = X(idx,:);
            X_ext = X_ext(1+obj.bottom_clip:end-obj.top_clip,:);
            obj.c1.phi = spline(X_ext(:,2), X_ext(:,1));
            obj.c1.phi_dot = fnder(obj.c1.phi,1);
            obj.c1.phi_ddot = fnder(obj.c1.phi,2);
            
            obj.step_count = 1;
            D = @(tau_info) obj.calcHolonomicCurveHelper(obj.c2.qp, obj.c2.qm, obj.c2.v, obj.fall_duration, tau_info(1), tau_info(2), obj.c2.impact_angle);
            tmp = fmincon(D,[obj.c1.pre_impact_angle; obj.c1.pre_impact_torque], [], [], [], [], [-pi/2, 0], [0, obj.tau_limit], [], options);
            obj.c2.pre_impact_angle = tmp(1);
            obj.c2.pre_impact_torque = tmp(2);
            obj.c2.tau_const = obj.getBestConstTau(obj.c2.qp, obj.c2.qm, obj.c2.v, obj.fall_duration, obj.c2.pre_impact_angle, obj.c2.pre_impact_torque);
            X = obj.getFallingCurve([obj.c2.qp; obj.c2.v], obj.fall_duration, [0; obj.c2.tau_const], obj.c2.pre_impact_angle, obj.c2.pre_impact_torque);
            [~,idx] = unique(X(:,2));
            X_ext = X(idx,:);
            X_ext = X_ext(1+obj.bottom_clip:end-obj.top_clip,:);
            obj.c2.phi = spline(X_ext(:,2), X_ext(:,1));
            obj.c2.phi_dot = fnder(obj.c2.phi,1);
            obj.c2.phi_ddot = fnder(obj.c2.phi,2);
            
            % Set back to original
            obj.step_count = 0;
        end
        
        function [v, w] = getImpactVelocities(obj, qm, qp, impact_angle, impact_velocity)
            
            rend = obj.calc_rend(obj.leg_length, obj.leg_length, qm(1), qm(2));
            rend = rend + [cos(impact_angle) * 0.01; sin(impact_angle) * 0.01];
            qm_pre = obj.calc_qd(obj.leg_length, obj.leg_length, rend(1), rend(2));
            w = unit(qm - qm_pre) * impact_velocity;
            
            % Post impact calculations
            De = obj.calc_De(obj.linertia(1), obj.linertia(2), obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2), qm(1), qm(2));
            E = obj.calc_E(obj.leg_length, obj.leg_length, qm(1), qm(2));
            dUde = obj.calc_dUde(obj.leg_length, qm(1));
            last_term = [eye(2); dUde];

            delta_F = -(E/De*E')\E*last_term;
            delta_qedot = De\E'*delta_F + last_term;
            T = [1 1; 0 -1]; % Relabelling
            qp_dot = [T zeros(2,2)] * (delta_qedot * w) * obj.lcurve.energy_loss;
            rend_dot = obj.calc_J(obj.leg_length, obj.leg_length, qp(1), qp(2)) * qp_dot;
            if (rend_dot(2) < 0)
                v = -qp_dot;
            else
                v = qp_dot;
            end
        end
        
        
        function dist = getBestConstTauHelper(obj, tau, qp, qm, v, dur, pre_impact_angle, pre_impact_torque)
            [X, fail] = obj.getFallingCurve([qp; v], dur, [0; tau], pre_impact_angle, pre_impact_torque);
            if (fail)
                dist = norm(X(end, 1:2) - qm') + 1e9;
            else
                dist = norm(X(end, 1:2) - qm');
            end
        end
        
        function tau_best = getBestConstTau(obj, qp, qm, v, dur, pre_impact_angle, pre_impact_torque)            
            if (obj.plot_curves)
                figure;
            end
            
            D = @(tau) obj.getBestConstTauHelper(tau, qp, qm, v, dur, pre_impact_angle, pre_impact_torque);
            
            tau_best = fminbnd(D,-obj.tau_limit,0);

            if (obj.plot_curves)
                X = obj.getFallingCurve([qp; v], dur, [0; tau_best], pre_impact_angle, pre_impact_torque);
                plot(X(:,1),X(:,2), 'color', [0.5,0,0,1]);
                hold on;
                
                tau_best = 0;
                closest_distance = 100000;
                for tau = -obj.tau_limit:0.005:0
                    X = obj.getFallingCurve([qp; v], dur, [0; tau], pre_impact_angle, pre_impact_torque);
                    if (obj.plot_curves)
                        plot(X(:,1),X(:,2), 'color', [0,0.5,0,0.3]);
                    end
                    dist = norm(X(end,1:2) - qm');
                    if (dist < closest_distance)
                        closest_distance = dist;
                        tau_best = tau;
                    end
                    if (obj.plot_curves)
                        hold on;
                    end
                end
                
                plot(qp(1), qp(2),'o', 'MarkerSize',5,'color','k');
                plot(qm(1), qm(2),'o', 'MarkerSize',5,'color','k');
                q1_range = 0:0.05:pi;
                plot(q1_range, -2 * q1_range + 2 * pi,'color','cyan');
                plot(q1_range, -2 * q1_range, 'color','cyan');
                
                % Plot 90 degree fall curve
                rend = obj.calc_rend(obj.leg_length, obj.leg_length, qm(1), qm(2));
                rend_range = [sin(pre_impact_torque); cos(pre_impact_torque)] * (0:0.01:0.05);
                qup = zeros(2, length(rend_range));
                for i = 1:length(rend_range)
                    qup(:,i) = obj.calc_qd(obj.leg_length, obj.leg_length, rend(1) + rend_range(2, i), rend(2) + rend_range(2, i));
                end
                plot(qup(1,:), qup(2,:));
                
                axis equal;
                grid minor;
                xlim([0, pi]);
                ylim([-pi, pi]);
                xlabel('q1');
                ylabel('q2');
                title(strcat('Best Tau: ', num2str(tau_best)));
                hold off;
            end
        end
        
        function fnplti(~, fnct)
            X = -pi:0.01:pi;
            Y = fnval(fnct, X);
            plot(Y, X, 'LineWidth',2);
        end
        
        function plotHolonomicCurve(obj, curve)
            obj.plotQField();
            hold on;
            obj.plotPField();
            obj.plotControllerSensitivityField();
            obj.plotFallingCurve(curve);

            plot(curve.qp(1), curve.qp(2),'o', 'MarkerSize',5,'color','k');
            plot(curve.qm(1), curve.qm(2),'o', 'MarkerSize',5,'color','k');
            
            quiver(curve.qp(1), curve.qp(2), curve.v(1), curve.v(2), 'LineWidth', 2, 'MaxHeadSize', 0.4);
            quiver(curve.qm(1), curve.qm(2), curve.w(1), curve.w(2), 'LineWidth', 2, 'MaxHeadSize', 0.4);
            
            obj.fnplti(curve.phi);
            
            title(strcat('Plot of sigma(theta) Tau: ', num2str(curve.tau_const)))
            xlabel('q1')
            ylabel('q2')
            axis equal
            xlim([0, pi])
            hold off;
        end
        
        function plotQField(obj)
            q1_range = 0:0.05:pi;
            q2_range = -pi:0.05:pi;
            [BX,BY] = meshgrid(q1_range,q2_range);
            
            BU = zeros(size(BX));
            BV = zeros(size(BY));

            % Dinv * B
            m1 = obj.lmass(1);
            m2 = obj.lmass(2);
            for i=1:size(BX,1)
                for j=1:size(BY,2)
                    temp = obj.calc_D(obj.linertia(1), obj.linertia(2), obj.leg_length, obj.lcom(1), obj.lcom(2), ...
                                    m1, m2, BY(i,j)) \ obj.B;
                    BU(i,j) = temp(1);
                    BV(i,j) = temp(2);
                end
            end   
            
%             quiver(obj.BX,obj.BY,obj.BU,obj.BV) %orbits
            streamslice(BX,BY,BU,BV,'color','cyan'); %orbits
            
            hold on;

            % Plot impact surfaces S+, S-
            plot(q1_range, -2 * q1_range + 2 * pi,'color','cyan');
            plot(q1_range, -2 * q1_range, 'color','cyan');
            xlim([0, pi]);
            ylim([-pi, pi]);
            
            hold off;
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

        function [X, fail] = getFallingCurve(obj, X_s, tend, tau, tau_stop_angle, pre_impact_torque)
            fail = 0;
            if nargin < 5
                tau = [0; 0];
            end
            if nargin < 6
                tau_stop_angle = -pi;
            end
            % Basically perform a fall in reverse, TODO, use ODE
            t = 0;
            
            % Run Tau until angle reached
            options = odeset('Events',@(t,x)obj.angle_min(t,x, tau_stop_angle), 'RelTol', 1e-2, 'AbsTol', 1e-5);
            [~, X1, te, xe, ie] = ode45(@(t, x) obj.step(t, x, tau), [t tend], X_s, options);
            if (ie(1) ~= 1)
                X = X1;
                fail = 1;
                return
            end     

            % Continue rest of movement with angle
            options = odeset('Events',@(t,x)obj.dist_to_floor(t,x), 'RelTol', 1e-2, 'AbsTol', 1e-5);
            [~, X2, ~, ~, ie] = ode45(@(t, x) obj.step(t, x, [0; pre_impact_torque]), [te tend], xe, options);
            if (ie(1) ~= 1 || length(ie) ~= 1)
                fail = 1;
            end
            X = [X1; X2];
        end

        function plotFallingCurve(obj, curve)
            X = obj.getFallingCurve([curve.qp; curve.v], 0.3, 1);
            quiver(X(:,1),X(:,2),X(:,3),X(:,4),'color',[1 1 0], 'markersize',1)
            xlabel('q1');
            ylabel('q2');
        end

        function plotControllerSensitivityField(obj)
            % Plot vector field inv(D)*B
            finity = 0.1;
            q1_range = 0:finity:pi;
            q2_range = -pi:2*finity:pi;
            [X1,X2] = meshgrid(q1_range,q2_range);

            R = zeros(size(X1));

            % Dinv * B
            m1 = obj.lmass(1);
            m2 = obj.lmass(2);
            for i=1:size(X2,1)
                for j=1:size(X2,2)
                    D = obj.calc_D(obj.linertia(1), obj.linertia(2), obj.leg_length, obj.lcom(1), obj.lcom(2), ...
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

            alpha 0.15
            hold on;
            plot(q1_range, -2 * q1_range + 2 * pi,'color','black');
            plot(q1_range, -2 * q1_range, 'color','black');
            xlim([0, pi]);
            ylim([-pi, pi]);
            title('Controller Sensitivity')
            xlabel('q1')
            ylabel('q2')
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
            
            % Inverse kinematics
            syms x y r q1d q2d qd real;
            q2d = acos((l1^2 + l2^2 - (x^2 + y^2))/l1/l2/2) - pi;
            q1d = atan2(y,x) + (pi - (pi + q2d))/2;
            qd = [q1d; q2d];
            
            % Add the matlab functions
            obj.calc_J = matlabFunction(renddot);
            obj.calc_D = matlabFunction(D);
            obj.calc_C = matlabFunction(C);
            obj.calc_P = matlabFunction(P);
            obj.calc_De = matlabFunction(De);
            obj.calc_E = matlabFunction(E);
            obj.calc_dUde = matlabFunction(dUde);
            obj.calc_EE = matlabFunction(EE);
            obj.calc_qd = matlabFunction(qd);
            obj.calc_rend = matlabFunction(rend);
        end
    end
end
