function P = getP_wrap(dI,dF,dt,d)

dth = (dF-dI)/dt;
l1=d.l1;
l2=d.l2;
l3=d.l3;
lc1 = d.lc1;
lc2 = d.lc2;
lc3 = d.lc3;
m1=d.m1;
m2=d.m2;
m3=d.m3;
I1=d.I1;
I2=d.I2;
I3=d.I3;

a1 = 0; a3 = 0;
Ir = Ir_fun(I1,I2,I3,a1,a3,l1,l2,lc1,lc2,lc3,m1,m2,m3);

P = Ir(end:end)*dth;
 
end