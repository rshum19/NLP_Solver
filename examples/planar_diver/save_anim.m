clear all; close all; clc;

% Load Results
%load('TLD_2pi_ht10_ND.mat')
load('TLDFig_4pi_ht10_D_p40.mat')
params = ThreeLinkDiver_Properties;

%v = VideoWriter('anim_OptSoln0.avi');
%open(v)
z = z';
for i=1:size(z,2)
    name = ['screenshot/TLD_D/TLDFig_4pi_ht10_D_',num2str(i)] ;
    ThreeLinkDiver_Draw(z(2,i),z(1,i),z(3,i),params)
    
    % Write time stamp
    ti = round(t(i),2);
    ti = strcat(num2str(ti),'s');
    s = text(0.48,-1.1,'t = ');
    s.FontSize = 30;
    s.FontWeight = 'bold';
    s = text(0.68,-1.1,ti);
    s.FontSize = 30;
    s.FontWeight = 'bold';

    box on
    set(gca,'XTick',[],'YTick',[]);
    set(gcf,'color','white')
    print(gcf,'-dpng',name)
    drawnow
    frame=getframe;
    %writeVideo(v,frame)
    pause(0.01);
end

%close(v);

% z=z';
% for i=1:size(z,2)
% name = ['ForThePaper/TLDFig_2pi_ht10_ND_',num2str(i)] ;
% ThreeLinkDiver_Draw(z(2,i),z(1,i),z(3,i),OCP.model.params);
% print(gcf,'-dpng',name)
% drawnow
% % frame=getframe;
% % writeVideo(v,frame)
% pause(0.01);
% end