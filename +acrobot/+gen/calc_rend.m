function rend = calc_rend(l1,l2,q1,q2)
%CALC_REND
%    REND = CALC_REND(L1,L2,Q1,Q2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    30-Mar-2020 18:45:52

t2 = q1+q2;
rend = [l1.*cos(q1)+l2.*cos(t2);l1.*sin(q1)+l2.*sin(t2)];
