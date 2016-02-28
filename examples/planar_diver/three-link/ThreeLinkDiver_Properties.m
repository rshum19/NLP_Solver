function d = ThreeLinkDiver_Properties()

d.ht = 10;
d.g = 9.81;
d.tf = sqrt(2*d.g*d.ht);
d.m1 = 4.55;  % hand
d.m2 = 45.88; % body
d.m3 = 13.85; % leg
d.I1 = 0.1579;
d.I2 = 1.9155;
d.I3 = 0.9753;
d.l1 = 0.66;
d.l2 = 0.553;
d.l3 = 0.837;
d.lc1 = 0.2441;
d.lc2 = 0.2369;
d.lc3 = 0.2976;

d.M = 64.28;
d.L_cm = 0.9829;
d.L = 2.0500;

end


%% Mass distribution
%%% Body - 50.80 || Leg - 15.98 || Hand - 4.96