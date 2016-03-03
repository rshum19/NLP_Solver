%clear all; close all; clc;
addpath('solution')
% Load data to compare
fprintf('--- Loading optimziation results data...')
data1 = load('threeLink_ornt_soln12.mat');
data2 = load('threeLink_ornt_soln10.mat');
fprintf('DONE\n')

% Plot result set #1
soln1 = data1.soln(end);
tGrid = soln1.grid.time;
t1 = linspace(tGrid(1),tGrid(end),100);
z1 = soln1.interp.state(t1);
u1 = soln1.interp.control(t1);
plot_results(t1,z1,u1,1);

% Plot result set #2
soln2 = data2.soln(2);
tGrid = soln2.grid.time;
t2 = linspace(tGrid(1),tGrid(end),1000);
z2 = soln2.interp.state(t2);
u2 = soln2.interp.control(t2);
plot_results(t2,z2,u2,2);

% figure(5)
% plot(t2,u2(1:2,:),'linewidth',2);
% xhandle = xlabel('time [sec]');
% yhandle = ylabel('radians/sec^2');
% title('Joint angle input')
% legend('u_1','u_2')
% set(gca,'Fontsize',10,'Fontweight','bold')
% set(xhandle,'Fontsize',12,'Fontname','Timesnewroman','Fontweight','bold')
% set(yhandle,'Fontsize',12,'Fontname','Timesnewroman','Fontweight','bold')

% Check dynamics
check_dyn_cst(soln2,data2.OCP)
