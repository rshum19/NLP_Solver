%% Dynamics of Quadrotor with suspended spring pendulum
function[dx] = odefun_dynamics(t,x,u,data)
% Dynamics of quadrotor suspended with load
% Constants
mL = data.params.mL ;
g = data.params.g ;
mQ = data.params.mQ ;
J = data.params.J ;
L = data.params.L ;
e1 = data.params.e1 ;
e2 = data.params.e2 ;
e3 = data.params.e3 ;
k = data.params.k;

nGrid = size(u,2);
dx = zeros(size(x));

   % Forces
   %f = (mL+ mQ)*g ; % just hovering at a place. 
   %M = [ 0;0;0];
   for i = 1:nGrid
       f = u(1,i);
       M = u(2:4,i);
       % Extracting States
       %------------------%
       xL = x(1:3,i) ;
       vL = x(4:6,i) ;
       q = x(7:9,i) ; 
       omega = x(10:12,i) ; 
       dq = vec_cross(omega, q);
       R = reshape(x(13:21), 3,3) ;
       Omega = x(22:24,i) ;
       if data.spring_flag
           l = x(25,i);
           dl = x(26,i);
       end
       b3 = R(:,3);


   % Equations of motion
   %-------------------%
       if data.spring_flag
           % Spring Length
           l_dot = dl;
           dl_dot = l*vec_dot(dl,dl) + (mQ + mL)*(L-l)*k/(mQ*mL) - vec_dot(q,f*b3)/mQ;

           % Load Attitude
           q_dot = dq;
           omega_dot = -(1/(mQ*l))*(vec_cross(q,f*b3) + 2*mQ*dl*omega);

           % Load position
           d2q = vec_cross(omega_dot,q) + vec_cross(omega,dq);
           xL_dot = vL;        
           vL_dot = 1/(mQ+mL) * ( f*b3 +  mQ*(dl_dot*q + 2*dl*dq + l*d2q) - (mQ+mL)*g*e3 ) ;

           % Quadrotor Attitude
           R_dot = R*hat_map(Omega) ;
           Omega_dot = inv(J)\( -vec_cross(Omega, J*Omega) + M ) ;

       elseif ~data.spring_flag
           % Load position
           xL_dot = vL;        
           vL_dot = 1/(mQ+mL) * ( (vec_dot(q,f*b3) - mQ*L*vec_dot(dq,dq))*q - (mQ+mL)*g*e3 ) ;

           % Load Attitude
           q_dot = dq;
           omega_dot = -(1/(mQ*L))*(vec_cross(q,f*b3));

           % Quadrotor Attitude
           R_dot = R*hat_map(Omega) ;
           Omega_dot = inv(J)\( -vec_cross(Omega, J*Omega) + M ) ;

       end

       % Computing dx
       %-------------%
       dx(:,i) = [xL_dot;
             vL_dot;
             q_dot;
             omega_dot;
             reshape(R_dot, 9,1) ;
             Omega_dot];
             %l_dot; 
             %dl_dot];
        
   end
   
end
