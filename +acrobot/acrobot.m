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
        
        g_func;
        
        % VHC Parameters
        B = [0; 1];
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
    end
    properties
        % Physical Parameters
        g = 9.81;
        
        % Mechanical Parameters
        leg_length = 0.335;
        foot_radius = 0.0075;

        % Curve Parameters
        beta = pi/9.5;
        vwscale = 1;
        impact_velocity = 2.5*pi; % To be computed using energy considerations

        % Debugging
        show_plot = 0;
    end
    
    methods
        function obj = acrobot()
            obj.robot.showdetails
            
            % Calculations
            for i = 1:2
                obj.mass(i) = obj.robot.Bodies{i}.Mass;
                obj.com(i) = norm(obj.robot.Bodies{i}.CenterOfMass(1));
                obj.inertia(i) = obj.robot.Bodies{i}.Inertia(1);
            end

            % Create Robot Equation handles
            obj.solveRoboticsEquation();
            
            % Solve for the curve
            obj.calcHolonomicCurve();
            
            % Calculate Q Curve
            obj.calculateQField();
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
        
        function calcHolonomicCurve(obj)
            obj.qm = [(pi - obj.beta)/2; obj.beta - pi]; % Joint angles pre impact
            obj.qp = [(pi + obj.beta)/2; pi - obj.beta]; % Joint angles post impact
            
            % Use the gravity potential field for calculation of w
            D = obj.calc_D(obj.leg_length, obj.com(1), obj.com(2), ...
                            obj.mass(1), obj.mass(2), obj.qm(2));
            P = obj.calc_P(obj.g, obj.leg_length, obj.com(1), obj.com(2), ...
                            obj.mass(1), obj.mass(2), obj.qm(1), obj.qm(2));
            J = obj.calc_J(obj.leg_length,obj.leg_length, obj.qm(1), obj.qm(2));

            min_angle = 2*pi;
            for angle = -pi:0.001:0 % Search for the angle of impact with the most natural fall
                wt = [cos(angle); sin(angle)] * obj.impact_velocity;
                qdot = J \ wt;

                C = obj.calc_C(obj.leg_length, obj.com(2), obj.mass(2), obj.qm(2), qdot(1), qdot(2));
                qddt = D \ (-C * qdot - P);
                if abs(angdiff(atan2(qddt(2), qddt(1)),angle)) < min_angle
                    obj.w = qdot;
                    min_angle = abs(angdiff(atan2(qddt(2), qddt(1)),angle));
                end
            end
            
            % Post impact calculations
            De = obj.calc_De(obj.leg_length, obj.com(1), obj.com(2), obj.mass(1), obj.mass(2), obj.qm(1), obj.qm(2));
            E = obj.calc_E(obj.leg_length, obj.leg_length, obj.qm(1), obj.qm(2));
            dUde = obj.calc_dUde(obj.leg_length, obj.qm(1));
            last_term = [eye(2); dUde];

            delta_F = -(E/De*E')\E*last_term;
            delta_qedot = De\E'*delta_F + last_term;
            T = [1 1; 0 -1]; % Relabelling
            qp_dot = [T zeros(2,2)] * (delta_qedot * obj.w);
            rend_dot = obj.calc_J(obj.leg_length, obj.leg_length, obj.qp(1), obj.qp(2)) * qp_dot;
            if (rend_dot(2) < 0)
                obj.v = -qp_dot;
            else
                obj.v = qp_dot;
            end
            
            % Use post fall curve
            X = obj.getFallingCurve([obj.qm; obj.w], 0.3, -1);
            obj.g_func = cscvn(X(:,1:2)');
        end
        
        function plotHolonomicCurve(obj)
            obj.plotQField();
            hold on;
            obj.plotPField();
            % obj.plotControllerSensitivityField();
            obj.plotFallingCurve();

            plot(obj.qp(1), obj.qp(2),'o', 'MarkerSize',5,'color','k');
            plot(obj.qm(1), obj.qm(2),'o', 'MarkerSize',5,'color','k');
            
            fnplt(obj.g_func);
            
            title('Plot of sigma(theta)')
            xlabel('q1')
            ylabel('q2')
            axis equal
            xlim([0, pi])
            hold off;
        end
        
        function testHolonomicCurve(obj)
            % Check Limit cycles
            
            syms q1 q2 q3 q4 real
            syms q1dot q2dot q1d_dot q2d_dot real
            syms theta g real

            q = [q1; q2];
            qdot = [q1dot; q2dot];

            % Centers of mass
            rc1 = obj.leg_length * [cos(q1); sin(q1)];
            rH = obj.leg_length * [cos(q1); sin(q1)];
            rc2 = rH + obj.leg_length * [cos(q1+q2); sin(q1+q2)];

            rc1_dot = simplify(jacobian(rc1,q)*qdot);
            rc2_dot = simplify(jacobian(rc2,q)*qdot);

            % Gravity vector
            U1 = obj.mass(1) * g * rc1(2);
            U2 = obj.mass(2) * g * rc2(2);
            U = U1 + U2;
            T1 = 0.5 * obj.mass(1) * (transpose(rc1_dot)*rc1_dot);
            T2 = 0.5 * obj.mass(2) * (transpose(rc2_dot)*rc2_dot);
            T = T1 + T2;

            jac_P = simplify(jacobian(U,q).');
            D = simplify(jacobian(T,qdot).');
            D = simplify(jacobian(D,qdot));

            % Christoffel coefficients
            syms C Q real
            C = sym(zeros(length(q)));
            Q = sym(zeros(length(q),length(q),length(q)));
            for k = 1:size(q)
                for j = 1:size(q)
                    for i = 1:size(q)
                        Q(i,j,k) = 0.5*(diff(D(k,j),q(i)) + diff(D(k,i),q(j)) - diff(D(i,j),q(k)));
                        C(k,j) = C(k,j) + simplify(Q(i,j,k)) * qdot(i);
                    end
                end
            end


            % Compute Psi1, Psi2 terms
            sigma_prime = sym('sigma_prime',[2 1]);
            sigma_d_prime = sym('sigma_d_prime',[2 1]);

            Bp = transpose(null(transpose(obj.B)));
            
            delta = Bp * D * sigma_prime;
            term1 = Bp * D * sigma_d_prime;

            Psi1 = - Bp * jac_P / delta; 

            term2 = 0;
            for i = 1:length(q)
                term2 = term2 + Bp(i) * transpose(sigma_prime) * Q(:,:,i) * sigma_prime;
            end

            Psi2 = -(1 / delta) * (term1 + term2);

            calc_psi1_numeric = matlabFunction(Psi1,'File','Psi1_func');
            calc_psi2_numeric = matlabFunction(Psi2,'File','Psi2_func');

            % Sub in values for sigma = q, sigma_prime, sigma_d_prime
            sigma_prime = fnder(obj.sigma,1);
            sigma_d_prime = fnder(obj.sigma,2);

            sigma_val = ppval(obj.sigma,obj.theta_f);
            sigma_prime_val = ppval(sigma_prime,obj.theta_f);

            % Calculate deltaz
            sigma_prime_plus = ppval(sigma_prime, obj.theta_start);
            sigma_prime_minus = ppval(sigma_prime, obj.theta_end);
            deltaz = (1 / (norm(sigma_prime_plus))^2) * (sigma_prime_plus' * obj.delta_qsdot * sigma_prime_minus); 

            M_0 = 1;
            V_0 = 0;
            x0 = [M_0;V_0];
            
            [~,x]=ode45(@(tt,x)obj.M_V_numeric_sys(tt,x,calc_psi1_numeric,calc_psi2_numeric, obj.sigma, sigma_prime, sigma_d_prime),obj.theta_f,x0);

            M_vals = x(:,1);
            V_vals = x(:,2);

            M_plus = M_vals(1);
            M_minus = M_vals(end);

            V_plus = V_vals(1);
            V_max = max(V_vals);
            V_minus = V_vals(end);

            disp(['M_minus: ',num2str(M_minus)])
            disp(['V_minus: ',num2str(V_minus)])
            disp(['V_max: ',num2str(V_max)])
            disp(['V_plus: ',num2str(V_plus)])

            % Existence test; want this less than 0
            temp_val = (M_plus / M_minus) * deltaz ^ 2;
            existence_test = (temp_val / (1 - temp_val)) * V_minus + V_max;
            disp(['Existence Test (should be less than 0): ',num2str(existence_test)]);
            assert(existence_test < 0);

            % Stability check; want this to be between 0 and 1
            stability_test = deltaz ^ 2 * (M_plus / M_minus);
            disp(['Stability Test (should be between 0 and 1): ',num2str(stability_test)]);
            assert(stability_test > 0 && stability_test < 1);

            % Plot the curves
            figure;
            plot(obj.theta_f,V_vals);
            hold on;
            plot(obj.theta_f,M_vals);
            title('V and M values')
            legend('V\_vals', 'M\_vals');
            xlabel('theta');
            grid minor;
            
            % Checking regularity
            % Is the VHC transversal everywhere to inv(D)*B? Check that sigma prime is
            % never tangent to inv(D)*B. If it is ever tangent, the matrix will be
            % singular; i.e., if the curve crosses 0, then the VHC is not regular
            for i=1:size(obj.theta_f,2)
                temp = inv(obj.calc_D(obj.leg_length, obj.com(1), obj.com(2), obj.mass(1), obj.mass(2),sigma_val(2,i))) * obj.B;
                detvals(i) = det([sigma_prime_val(:,i) temp]);
                assert(detvals(i) < 0);
            end
            
            figure;
            plot(obj.theta_f,detvals);
            title('Regularity check');
            xlabel('theta');
            ylabel('detvals');
            grid minor;
        end
        
        function dxdt = M_V_numeric_sys(~, theta,x,Psi1,Psi2, sigma, sigma_prime, sigma_d_prime)
            % M = x(1);
            % V = x(2);
            sigma_num = ppval(sigma,theta);
            sigma_prime_val = ppval(sigma_prime,theta);
            sigma_d_prime_val = ppval(sigma_d_prime, theta);

            q1 = sigma_num(1);
            q2 = sigma_num(2);
            sigma_prime1 = sigma_prime_val(1);
            sigma_prime2 = sigma_prime_val(2);
            sigma_d_prime1 = sigma_d_prime_val(1);
            sigma_d_prime2 = sigma_d_prime_val(2);

            Psi1_num = Psi1(q1,q2,sigma_prime1,sigma_prime2);
            Psi2_num = Psi2(q2,sigma_prime1,sigma_prime2,sigma_d_prime1,sigma_d_prime2);

            dxdt = [-2*Psi2_num 0;-Psi1_num 0]*x;
        end
        
        function calculateQField(obj)
            % Plot vector field inv(D)*B
            q1_range = 0:0.05:pi;
            q2_range = -pi:0.05:pi;
            [obj.BX,obj.BY] = meshgrid(q1_range,q2_range);
            
            obj.BU = zeros(size(obj.BX));
            obj.BV = zeros(size(obj.BY));

            % Dinv * B
            m1 = obj.mass(1);
            m2 = obj.mass(2);
            for i=1:size(obj.BX,1)
                for j=1:size(obj.BY,2)
                    temp = obj.calc_D(obj.leg_length, obj.com(1), obj.com(2), ...
                                    m1, m2, obj.BY(i,j)) \ obj.B;
                    obj.BU(i,j) = temp(1);
                    obj.BV(i,j) = temp(2);
                end
            end            
        end
        
        function plotQField(obj)
            q1_range = 0:0.05:pi;
            q2_range = -pi:0.05:pi;
            
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
        
        function plotPField(obj)
            % Plot vector field inv(D)*B
            finity = 0.2;
            q1_range = 0:finity:pi;
            q2_range = -pi:2*finity:pi;
            [X1,X2] = meshgrid(q1_range,q2_range);
            
            R = zeros(size(X1));
            Z = zeros(size(X2));

            % Dinv * B
            m1 = obj.mass(1);
            m2 = obj.mass(2);
            for i=1:size(X2,1)
                for j=1:size(X2,2)
                    D = obj.calc_D(obj.leg_length, obj.com(1), obj.com(2), ...
                                    m1, m2, X2(i,j));
                    P = obj.calc_P(obj.g, obj.leg_length, obj.com(1), obj.com(2), ...
                                    m1, m2, X1(i,j), X2(i,j));
                    temp = D \ -P;
                    R(i,j) = temp(1);
                    Z(i,j) = temp(2);
                end
            end
            
            quiver(X1,X2,R,Z,'color',[1 0 1])
%             hold on;
%             h = streamline(X1,X2,-R,-Z,obj.qm(1),obj.qm(2));
%             set(h,'Color','red');
%             h = streamline(X1,X2,-R,-Z,obj.qp(1),obj.qp(2));
%             set(h,'Color','red');
        end

        function X = getFallingCurve(obj, X_s, tend, dir)
            % Basically perform a fall in reverse, TODO, use ODE
            tstep = 0.005;
            s = 1;

            % Prefalling phase
            X = zeros(length(0:tstep:tend),4);
            X(1,:) = X_s; % End state
            for t = tstep:tstep:tend
                D = obj.calc_D(obj.leg_length, obj.com(1), obj.com(2), ...
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

        function plotFallingCurve(obj)
            % Basically perform a fall in reverse, TODO, use ODE
            X = obj.getFallingCurve([obj.qm; obj.w], 0.3, -1);
            quiver(X(:,1),X(:,2),X(:,3),X(:,4),'color',[0 1 0])
            xlabel('q1');
            ylabel('q2');

            hold on;

            X = obj.getFallingCurve([obj.qp; obj.v], 0.3, 1);
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
            m1 = obj.mass(1);
            m2 = obj.mass(2);
            obj.calc_P
            for i=1:size(X2,1)
                for j=1:size(X2,2)
                    D = obj.calc_D(obj.leg_length, obj.com(1), obj.com(2), ...
                                    m1, m2, X2(i,j));
                    P = obj.calc_P(obj.g, obj.leg_length, obj.com(1), obj.com(2), ...
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
            syms l1 lc1 l2 lc2 real
            syms m1 m2 g real

            q = [q1; q2];
            qdot = [q1dot; q2dot];
            qddot = [q1ddot; q2ddot];

            % Positions
            rc1 = (l1 - lc1) * [cos(q1); sin(q1)];
            rH = l1 * [cos(q1); sin(q1)];
            rc2 = rH + lc2 * [cos(q1+q2); sin(q1+q2)];
            rend = rH + l2 * [cos(q1+q2); sin(q1+q2)];

            rc1dot = simplify(jacobian(rc1,q) * qdot);
            rc2dot = simplify(jacobian(rc2,q) * qdot);
            renddot = simplify(jacobian(rend,q));

            % Kinetic and Potential Energy
            T1 = 0.5 * m1 * (rc1dot' * rc1dot);
            T2 = 0.5 * m2 * (rc2dot' * rc2dot);
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
            Te1 = 0.5 * m1 * (rc1edot' * rc1edot);
            Te2 = 0.5 * m2 * (rc2edot' * rc2edot);
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
