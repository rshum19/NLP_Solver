function [ data ] = params()

% System Constants
%-----------------%
data.params.mQ = 0.5 ;
data.params.mL = 0.087 ;
data.params.J = [2.32e-3,0,0;0,2.32e-3,0;0,0,4e-3] ;
data.params.g = 9.81 ;
data.params.e1 = [1;0;0] ;
data.params.e2 = [0;1;0] ;
data.params.e3 = [0;0;1] ;
data.params.k = (data.params.mL*data.params.g)/0.01; % spring constant
data.params.L = 0.5; % spring rest length
data.params.c = 01; % Damping coefficient
data.spring_flag = 0;


end

