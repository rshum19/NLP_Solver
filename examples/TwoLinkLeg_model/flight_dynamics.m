function [ t_flight, dY ] = flight_dynamics( Y0)

g = 9.81;

% Flight time
t_flight = sqrt(2*(Y0)/g);  % Flight time [s]

% Impact velocity
dY =  Y0 - g*t_flight;      % Y-velocity @ impact/apex

end

