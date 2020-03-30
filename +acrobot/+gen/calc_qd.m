function qd = calc_qd(l1,l2,x,y)
%CALC_QD
%    QD = CALC_QD(L1,L2,X,Y)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    30-Mar-2020 18:45:52

t2 = l1.^2;
t3 = l2.^2;
t4 = x.^2;
t5 = y.^2;
t6 = 1.0./l1;
t7 = 1.0./l2;
t8 = -t4;
t9 = -t5;
t10 = t2+t3+t8+t9;
t11 = (t6.*t7.*t10)./2.0;
t12 = acos(t11);
qd = [t12.*(-1.0./2.0)+pi./2.0+atan2(y,x);t12-pi];
