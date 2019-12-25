classdef acrobot < handle
    
    properties
        % Mechanical Parameters
        robot = importrobot("acrobot_description/models/acrobot_simple.urdf");
        leg_length = 0.4; % 0.348;
        foot_radius = 0.0075;
        
        % VHC Parameters
        B = [0; 1];

        beta = pi/4; % Angle between legs on impact
        vwscale = 1;
        kappa = 2.1;
        
        % Function Handles
        calc_D;
        calc_C;
        calc_P;
        calc_b;
        calc_qddot;
        
        calc_De;
        calc_E;
        calc_dUde;
        
        % Output curves properties
        delta_qsdot;
        sigma;
        theta_f;
        theta_start;
        theta_end;
        g_func;
        
        % Convenience values
        mass = zeros(2,1);
        com = zeros(2,1);
        inertia = zeros(2,1);
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
            
            % Tests
            obj.testHolonomicCurve();
        end
        
        function calcHolonomicCurve(obj)
            w = [cos(obj.kappa); sin(obj.kappa)]; 

            qm = [(pi - obj.beta)/2; obj.beta - pi]; % Joint angles pre impact
            qp = [(pi + obj.beta)/2; pi - obj.beta]; % Joint angles post impact

            De = obj.calc_De(obj.leg_length, obj.com(1), obj.com(2), obj.mass(1), obj.mass(2), qm(1), qm(2)); 
            E = obj.calc_E(obj.leg_length, obj.leg_length, qm(1), qm(2));
            dUde = obj.calc_dUde(); 
            
            delF = -((E / De) * transpose(E)) \ E * [eye(2); dUde];
            delta_qedot = (De \ transpose(E)) * delF + [eye(2); dUde];
            obj.delta_qsdot = [[1 1; 0 -1] zeros(2,2)] * delta_qedot;

            v = (obj.delta_qsdot) * w / obj.vwscale;
            v = (v / norm(v));

            % Line parameters
            r0 = 0.2;   % start to qp
            r1 = 0.55;  % qp to qa
            t0 = 0.05;  % qb to qm
            t1 = 0.2;   % qm to end

            % Number of samples
            r0_samples = 2; % start to qp
            r1_samples = 3; % qp to qa
            s_samples = 10; % qa to qb
            t0_samples = 6; % qb to qm
            t1_samples = 2; % qm to end

            % Truncating indices
            % Note: without truncation there is overlap at qa and qb
            r1_truncate = 1;        % Truncate before qa
            s_start_truncate = 4;   % Truncate after qa
            s_end_truncate = 5;     % Truncate before qb
            t0_truncate = 3;        % Truncate after qb

            % Creating the curve
            r0_vec = linspace(-r0,0,r0_samples);
            r1_vec = linspace(0,r1,r1_samples);
            r = [r0_vec(1:end-1), r1_vec]; % start to qa
            t0_vec = linspace(-t0,0,t0_samples);
            t1_vec = linspace(0,t1,t1_samples);
            t = [t0_vec(1:end-1), t1_vec]; % qb to end

            % Points on the curve
            q_start = qp + v * r;
            q_end = qm + w * t;
            s = linspace(r1_vec(end),1-abs(t0_vec(1)),s_samples); %qa to qb
            q_mid = q_start(:,end) + (q_end(:,1) - q_start(:,end))/(s(end) - s(1)) * (s-s(1));
            q_rough = [q_start, q_mid, q_end];

            % Middle section
            theta_rough = [r, s, t + 1]; 

            % Find theta values for start and end of curve (intersecting S+, S-)
            obj.theta_start = theta_rough(r == 0);
            obj.theta_end = theta_rough(find(t == 0) + size(r,2) + size(s,2));

            % Create breakpoints for spline
            r_bar = r(1:end - r1_truncate);
            t_bar = t(t0_truncate+1:end);
            s_bar = s(s_start_truncate+1:end-s_end_truncate);

            theta_truncated = [r_bar, s_bar, 1 + t_bar];
            q_truncated = [q_start(:,1:end-r1_truncate), q_mid(:,s_start_truncate+1:end-s_end_truncate), q_end(:,t0_truncate+1:end)];

            % The curve
            obj.sigma = spline(theta_truncated, [v q_truncated w]);
            obj.theta_f = linspace(theta_rough(1),theta_rough(end),800);
            
            q_f = ppval(obj.sigma,obj.theta_f);
            obj.g_func = spline(q_f(1,:),q_f(2,:)); %q2 as function of q1
            yy = linspace(q_f(1,1), q_f(1,end));
            zz = ppval(obj.g_func, yy);
            
            % Plotting the curve
            figure;
            obj.plotQField();
            hold on;
            
            plot(q_truncated(1,:), q_truncated(2,:), '*', 'color', 'y');
            plot(qp(1), qp(2),'.', 'MarkerSize',13,'color','g');
            plot(qm(1), qm(2),'.', 'MarkerSize',13,'color','g');
            plot(q_start(1,end), q_start(2,end),'.', 'MarkerSize',10,'color','g');
            plot(q_end(1,1), q_end(2, 1), '.','MarkerSize',10,'color','g');

            plot(q_rough(1,:), q_rough(2,:), '+', 'color', 'r');
            plot(yy, zz, '.');

            title('Plot of sigma(theta)')
            xlabel('q1')
            ylabel('q2')
            axis square
            hold off;
        end
        
        function testHolonomicCurve(obj)
            % Check Limit cycles
            
            syms q1 q2 q3 q4 real
            syms q1dot q2dot q1d_dot q2d_dot real
            syms theta g real
            
            g = 9.81;
            
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
        
        function plotQField(obj)
            % Plot vector field inv(D)*B
            q1_range = 0:0.05:pi;
            q2_range = -2*pi:0.05:2*pi;
            [X1,X2] = meshgrid(q1_range,q2_range);
            
            R = zeros(size(X1));
            Z = zeros(size(X2));

            % Dinv * B
            m1 = obj.mass(1);
            m2 = obj.mass(2);
            for i=1:size(X2,1)
                for j=1:size(X2,2)
                    temp = obj.calc_D(obj.leg_length, obj.com(1), obj.com(2), ...
                                    m1, m2, X2(i,j)) \ obj.B;
                    R(i,j) = temp(1);
                    Z(i,j) = temp(2);
                end
            end
            
            streamslice(X1,X2,R,Z,'color','cyan') %orbits
            
            hold on;
            

            % Plot impact surfaces S+, S-
            plot(q1_range, -2 * q1_range(1,:) + 2 * pi,'color','cyan');
            plot(q1_range, -2 * q1_range, 'color','cyan');
            
            hold off;
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
            rc1 = lc1 * [cos(q1); sin(q1)];
            rH = l1 * [cos(q1); sin(q1)];
            rc2 = rH + lc2 * [cos(q1+q2); sin(q1+q2)];

            rc1dot = simplify(jacobian(rc1,q) * qdot);
            rc2dot = simplify(jacobian(rc2,q) * qdot);

            % Kinetic and Potential Energy
            T1 = 0.5 * m1 * (transpose(rc1dot) * rc1dot);
            T2 = 0.5 * m2 * (transpose(rc2dot) * rc2dot);
            T = T1 + T2;
            U1 = m1 * g * rc1(2);
            U2 = m2 * g * rc2(2);
            U = U1 + U2;
            L = simplify(T - U);
            dLdq = jacobian(L,q)';
            dLdqdot = jacobian(L,qdot)';
            ddtdLdqdot = jacobian(dLdqdot,q) * qdot + jacobian(dLdqdot,qdot) * qddot;
            Tau = simplify(ddtdLdqdot - dLdq);

            % Find the C, D, P Matrix
            syms C D P real
            [D, b] = equationsToMatrix(Tau, [q1ddot; q2ddot]);
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
            rc1d = rc1 + [q3; q4];
            rc2d = rc2 + [q3; q4];

            rc1ddot = simplify(jacobian(rc1d, qe) * qedot);
            rc2ddot = simplify(jacobian(rc2d, qe) * qedot);

            % Kinetic and Potential Energy
            Td1 = 0.5 * m1 * (transpose(rc1ddot) * rc1ddot);
            Td2 = 0.5 * m2 * (transpose(rc2ddot) * rc2ddot);
            Td = Td1 + Td2 ;
            Ud1 = m1 * g * rc1d(2);
            Ud2 = m2 * g * rc2d(2);
            Ud = Ud1 + Ud2;
            Le = simplify(Td - Ud);

            % Finding the EOM
            dLedq = transpose(jacobian(Le,qe));
            dLedqdot = transpose(jacobian(Le,qedot));
            ddtdLedqdot = simplify(jacobian(dLedqdot, qe) * qedot + jacobian(dLedqdot, qedot) * qeddot);
            Taue = simplify(ddtdLedqdot - dLedq);

            % Solve for D matrix
            [De, ~] = equationsToMatrix(Taue, qeddot);

            % Upsilons
            e = [q3;q4];
            E = simplify(jacobian(rc2d, qe));
            dUde = simplify(jacobian(e,q));
            
            % Add the matlab functions
            obj.calc_qddot = matlabFunction(simplify(inv(D) * [0; 1]));
            obj.calc_D = matlabFunction(D);
            obj.calc_C = matlabFunction(C);
            obj.calc_b = matlabFunction(b);
            obj.calc_P = matlabFunction(P);
            obj.calc_De = matlabFunction(De);
            obj.calc_E = matlabFunction(E);
            obj.calc_dUde = matlabFunction(dUde);
        end    
    end
end
