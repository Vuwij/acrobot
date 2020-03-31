function C = calc_C(l1,lc2,m2,q2,q1dot,q2dot)
%CALC_C
%    C = CALC_C(L1,LC2,M2,Q2,Q1DOT,Q2DOT)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    30-Mar-2020 20:27:27

t2 = sin(q2);
t3 = l1.*lc2.*m2.*q1dot.*t2;
t4 = l1.*lc2.*m2.*q2dot.*t2;
t5 = -t4;
C = reshape([t5,t3,-t3+t5,0.0],[2,2]);
