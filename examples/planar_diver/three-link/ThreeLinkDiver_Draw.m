function ThreeLinkDiver_Draw(alpha1,theta2,alpha3,d)
figure(101)
clf
%%% Height Info
g = d.g;
ht = d.ht;

%%% Model dimensions %%%
l1=d.l1;
l2=d.l2;
l3=d.l3;
lc1 = d.lc1;
lc2 = d.lc2;
lc3 = d.lc3;
L = d.L;
Lc1 = d.L_cm; 

%%%%%% Width %%%
r1 = 0.42/3; %% hand
r2 = 0.56/3; %% body 
r3 = 0.46/3; %% leg

%%%% Draw Ground
%     plot([-10 10], -ht*[1 1], 'k', 'LineWidth', 2) ;
 
%%%% Segment 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
  %% Draw Body 
   b = [0;0]; %% body geometric center 
   ellipse(r2/2,l2/2,-theta2,b(1),b(2),'r');
hold on  
  %% Mark CoM   
   b_cm = b + R(-theta2)*[0;((l2/2) - lc2)];  %% body center of mass
   plot(b_cm(1),b_cm(2),'o','MarkerSize',20,'MarkerFaceColor','k')

    
%%%% Segment 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
  %% Draw Hand 
   h = b + R(-theta2)*[0;(l2/2)] + R(-theta2-alpha1)*[0;l1/2] ; %% hand geometric center 
   ellipse(r1/2,l1/2,-theta2-alpha1,h(1),h(2),'g');
  
  %% Mark CoM   
   h_cm = h + R(-theta2-alpha1)*[0;-((l1/2) - lc1)];  %% hand center of mass
   plot(h_cm(1),h_cm(2),'o','MarkerSize',20,'MarkerFaceColor','k')

%%%% Segment 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
  %% Draw Leg
   l = b - R(-theta2)*[0;(l2/2)] - R(-theta2+alpha3)*[0;l3/2] ; %% leg geometric center 
   %cl = [0.9763,0.9831,0.0538];
   ellipse(r3/2,l3/2,-theta2+alpha3,l(1),l(2),'b');
  
  
  %% Mark CoM   
   l_cm = l + R(-theta2+alpha3)*[0;((l3/2) - lc3)];  %% leg center of mass
   plot(l_cm(1),l_cm(2),'o','MarkerSize',20,'MarkerFaceColor','k')

%%%% Final Segment %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   

  %% Mark overall body CoM   
   D_cm = (d.m2*b_cm + d.m1*h_cm + d.m3*l_cm)/(d.m1+d.m2+d.m3);
   plot(D_cm(1),D_cm(2),'v','MarkerSize',20,'MarkerFaceColor','m')
   
hold off
axis([-0.55*L 0.55*L -0.65*L 0.65*L])
camlight
end


