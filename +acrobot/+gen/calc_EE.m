function EE = calc_EE(g,i1,i2,l1,lc1,lc2,m1,m2,q1,q2,q1dot,q2dot)
%CALC_EE
%    EE = CALC_EE(G,I1,I2,L1,LC1,LC2,M1,M2,Q1,Q2,Q1DOT,Q2DOT)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    30-Mar-2020 18:45:52

t2 = cos(q1);
t3 = sin(q1);
t4 = q1+q2;
t5 = q1dot.^2;
t9 = -lc1;
t6 = cos(t4);
t7 = l1.*t3;
t8 = sin(t4);
t11 = l1+t9;
t10 = lc2.*t8;
t12 = t11.^2;
t13 = t7+t10;
EE = (i1.*t5)./2.0+(m1.*(t2.^2.*t5.*t12+t3.^2.*t5.*t12))./2.0+(i2.*(q1dot+q2dot).^2)./2.0+(m2.*((q1dot.*(l1.*t2+lc2.*t6)+lc2.*q2dot.*t6).^2+(q2dot.*t10+q1dot.*t13).^2))./2.0+g.*m2.*t13+g.*m1.*t3.*t11;