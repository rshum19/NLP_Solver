function [ GRF ] = getGRF( x, dx)

Ac = 2;
a = 1000000;
b = 1.38;
c = 20000;
d = 0.75;
e = 1;


GRF = nan(length(x),1);
x = -x;
dx = -dx;
for i = 1:length(x)
    if(x(i) > 0)
        GRF(i) = Ac*(a*x(i)^b + c*(x(i)^d)*dx(i)^e);
    else
        GRF(i) = 0;
    end
end

end

