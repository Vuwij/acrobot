function P = calc_P(g,l1,lc1,lc2,m1,m2,q1,q2)
%CALC_P
%    P = CALC_P(G,L1,LC1,LC2,M1,M2,Q1,Q2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    30-Mar-2020 18:45:51

t2 = cos(q1);
t3 = q1+q2;
t4 = cos(t3);
P = [g.*m2.*(l1.*t2+lc2.*t4)+g.*m1.*t2.*(l1-lc1);g.*lc2.*m2.*t4];
