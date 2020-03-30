function renddot = calc_J(l1,l2,q1,q2)
%CALC_J
%    RENDDOT = CALC_J(L1,L2,Q1,Q2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    30-Mar-2020 06:02:42

t2 = q1+q2;
t3 = cos(t2);
t4 = sin(t2);
t5 = l2.*t3;
t6 = l2.*t4;
t7 = -t6;
renddot = reshape([t7-l1.*sin(q1),t5+l1.*cos(q1),t7,t5],[2,2]);
