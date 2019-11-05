classdef acrobot
    
    properties
        % Mechanical Parameters
        robot = importrobot("acrobot_description/models/acrobot.urdf");
        leg_length = 0.348;
        foot_radius = 0.0075;
        
        % VHC Parameters
        B = [0; 1];
        beta = pi/4; % Angle between legs on impact
        vwscale = 1;
        kappa = 2.1;
    end
    
    methods
        function mass = mass(obj, num)
            mass = obj.robot.Bodies{num}.Mass;
        end
        
        function com = com(obj, num)
            com = norm(obj.robot.Bodies{num}.CenterOfMass(1));
        end
        
        function inertia = inertia(obj, num)
            inertia = obj.robot.Bodies{num}.Inertia(1);
        end
        
        function calcHolonomicCurve(obj)
            [calc_D, calc_De, calc_E, calc_dUde] = obj.symbols();

            w = [cos(obj.kappa); sin(obj.kappa)]; 

            qm = [(pi - obj.beta)/2; obj.beta - pi]; % Joint angles pre impact
            qp = [(pi + obj.beta)/2; pi - obj.beta]; % Joint angles post impact

            De = calc_De(obj.leg_length, obj.leg_length/2, obj.leg_length/2, obj.mass(1), obj.mass(2), qm(1), qm(2)); 
            E = calc_E(obj.leg_length, obj.leg_length, qm(1), qm(2));
            dUde = calc_dUde(); 
            
            delF = -((E / De) * transpose(E)) \ E * [eye(2); dUde];
            delta_qedot = (De \ transpose(E)) * delF + [eye(2); dUde];
            delta_qsdot = [[1 1; 0 -1] zeros(2,2)] * delta_qedot;

            v = (delta_qsdot) * w / obj.vwscale;
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
            q_mid = q_start(:,end) + (q_end(:,1) - q_start(:,end))/(s(end) - s(1)) * s-s(1);
            q_rough = [q_start, q_mid, q_end];

            % Middle section
            theta_rough = [r, s, t + 1]; 

            % Plotting
            test_plot = figure;
            hold on
            plot(qp(1), qp(2),'.', 'MarkerSize',13,'color','k');
            plot(qm(1), qm(2),'.', 'MarkerSize',13,'color','g');
            plot(q_rough(1,:), q_rough(2,:), 'o', 'color', 'b')

            % Find theta values for start and end of curve (intersecting S+, S-)
            theta_start = theta_rough(r == 0);
            theta_end = theta_rough(find(t == 0) + size(r,2) + size(s,2));

            % Create breakpoints for spline
            r_bar = r(1:end - r1_truncate);
            t_bar = t(t0_truncate+1:end);
            s_bar = s(s_start_truncate+1:end-s_end_truncate);


            theta_truncated = [r_bar, s_bar, 1+t_bar];
            q_truncated= [q_start(:,1:end-r1_truncate), q_mid(:,s_start_truncate+1:end-s_end_truncate), q_end(:,t0_truncate+1:end)];

            figure(test_plot);
            plot(q_truncated(1,:), q_truncated(2,:), '*');

            %The curve
            sigma = spline(theta_truncated, [v q_truncated w]);

            test = linspace(theta_start,theta_end,800);
            test2 = ppval(sigma,test);
            figure(test_plot)
            plot(test2(1,:), test2(2,:));

            % Plot the Rough solution curve

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
                    temp = calc_D(obj.leg_length, obj.leg_length/2, obj.leg_length / 2, ...
                                    m1, m2, X2(i,j)) \ obj.B;
                    R(i,j) = temp(1);
                    Z(i,j) = temp(2);
                end
            end

            VHC_plot = figure('Name','VHC_plot');
            hold on
            axis square
            title('Plot of sigma(theta)')
            xlabel('q1')
            ylabel('q2')

            streamslice(X1,X2,R,Z) %orbits

            % Mark q+, q-
            plot(qp(1), qp(2),'.', 'MarkerSize',10,'color','r');
            plot(qm(1), qm(2), '.','MarkerSize',10,'color','g');

            % Plot impact surfaces S+, S-
            plot(q1_range, -2 * q1_range(1,:) + 2 * pi) 
            plot(q1_range, -2 * q1_range)


            figure(VHC_plot);
            hold on

            % Plot q_a, q_b
            plot(q_start(1,end), q_start(2,end),'.', 'MarkerSize',10,'color','k');
            plot(q_end(1,1), q_end(2, 1), '.','MarkerSize',10,'color','k');

            ax1 = gca;
            VHC_plot_final_search = figure();
            ax2 = copyobj(ax1, VHC_plot_final_search);
            title('Searching for final curve');


            % Plot the new curve
            figure(VHC_plot);
            theta_final = linspace(theta_start,theta_end,1000);
            q_testing = ppval(sigma,theta_final);
            plot(q_testing(1,:), q_testing(2,:), 'k'); %Check that cuve matches

        end
        
        function [calc_D, calc_De, calc_E, calc_dUde] = symbols(obj)
            % Equations of Motion
            
            syms q1 q2 q1dot q2dot q1ddot q2ddot 
            syms l1 lc1 l2 lc2 
            syms m1 m2 g 

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
            Td1 = 0.5 * m1 * (transpose(rc1dot) * rc1dot);
            Td2 = 0.5 * m2 * (transpose(rc2dot) * rc2dot);
            Td = Td1 + Td2;
            Ud1 = m1 * g * rc1(2);
            Ud2 = m2 * g * rc2(2);
            Ud = Ud1 + Ud2;
            L = simplify(Td - Ud);
            dLdq = jacobian(L,q)';
            dLdqdot = jacobian(L,qdot)';
            ddtdLdqdot = jacobian(dLdqdot,q) * qdot + jacobian(dLdqdot,qdot) * qddot;
            Tau = simplify(ddtdLdqdot - dLdq);

            % Find the D Matrix
            [D, ~] = equationsToMatrix(Tau, [q1ddot; q2ddot]);

            % Impact Map
            % Solving for EOM and impact map following
            % "Feedback Control of Dynamic Bipedal Robot Locomotion" by Grizzle, p. 55
            % Based on the 3 link model on p. 67 and the paper "Asymptotically Stable
            % Walking for Biped Robots: Analysis via Systems with Impulse Effects"

            syms q3 q4 q3dot q4dot q3ddot q4ddot

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
            
            calc_D = matlabFunction(D);
            calc_De = matlabFunction(De);
            calc_E = matlabFunction(E);
            calc_dUde = matlabFunction(dUde);

        end    
    end
end
