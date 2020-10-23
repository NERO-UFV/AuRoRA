%% Rotina para plotar experimento de trajetória do drone com apoio do optitrack
clear all
close all
clc

%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Declare robot
% A = ArDrone;

%% Load data
% Data format: [pPos.Xd(1:12) pPos.X(13:24) time(25)]   
%    
% XX = load('drone_position_20180724T150738.txt')';
XX = load('drone_position_20180724T150738.txt')';
Xtil = XX(1:12,:) - XX(13:24,:);

%% Draw results
line_d  = 1;      % desired variables linewidth
line_r  = 0.5;    % real variables linewidth
line_e  = 0.8;    % erros line width
color_d = 'b--';  % line color desired variables
color_r = 'r';    % line color real variables
color_p = 'bx';   % line color desired points 
% --------------------------------------------------------------
%% Trajectory 2D - superior view
figure
% axis equal

plot(XX(1,:)',XX(2,:)',color_p,'MarkerSize',10,'LineWidth',2),hold on;
plot(XX(13,:)',XX(14,:)',color_r,'LineWidth',1);
plot(XX(13,1),XX(14,1),'ko','MarkerSize', 9,'LineWidth',2);
plot(XX(13,end),XX(14,end),'k*','MarkerSize', 9,'LineWidth',2);
xlabel('$x$ [m]','interpreter','Latex')
ylabel('$y$ [m]','interpreter','Latex')
% lt1 = legend('Desired','Real');
lt1 = legend('Desired','Traveled','Start','End');
set(lt1,'Interpreter','latex');
% box on
axis ([-2 2 -2 2])
grid on

%% Trajectory 3D
figure
axis equal
plot3(XX(1,:)',XX(2,:)',XX(3,:)',color_p,'MarkerSize',10,'LineWidth',2),hold on;
plot3(XX(13,:)',XX(14,:)',XX(15,:)',color_r,'LineWidth',1)
plot3(XX(13,1),XX(14,1),XX(15,1),'ko','MarkerSize', 9,'LineWidth',2);
plot3(XX(13,end),XX(14,end),XX(15,end),'k*','MarkerSize', 9,'LineWidth',2);

xlabel('$x$ [m]','interpreter','Latex')
ylabel('$y$ [m]','interpreter','Latex')
zlabel('$z$ [m]','interpreter','Latex')
% lt2 = legend('Desired','Real');
lt2 = legend('Desired','Traveled','Start','End');
set(lt2,'Interpreter','latex');
box on
grid on

%% --------------------------------------------------------------
% Euler angles - graus
figure
subplot(311)
plot(XX(end,:),XX(4,:)'*180/pi,color_d,'LineWidth',line_d),hold on;  % roll
plot(XX(end,:),XX(16,:)'*180/pi,color_r,'LineWidth',line_r)  % roll
le1 = legend('Desired','Real');
xlabel('Time [s]','interpreter','Latex')
ylabel('$\phi$ [$^o$]','interpreter','Latex')
% axis([0 XX(end,end) min(min(XX([4 16],:))*180/pi) max(max(XX([4 16],:))*180/pi)])
axis([0 XX(end,end) -5 5])
grid

subplot(312) % pitch
plot(XX(end,:),XX(5,:)'*180/pi,color_d,'LineWidth',line_d),hold on;  
plot(XX(end,:),XX(17,:)'*180/pi,color_r,'LineWidth',line_r)  
le2 = legend('Desired','Real');
xlabel('Time [s]','interpreter','Latex')
ylabel('$\theta$ [$^o$]','interpreter','Latex')
% axis([0 XX(end,end) min(min(XX([5 17],:))*180/pi) max(max(XX([5 17],:))*180/pi)])
axis([0 XX(end,end) -5 5])
grid

subplot(313) % yaw
plot(XX(end,:),XX(6,:)'*180/pi,color_d,'LineWidth',line_d),hold on;  
plot(XX(end,:),XX(18,:)'*180/pi,color_r,'LineWidth',line_r)  
le3 = legend('Desired','Real');
xlabel('Time [s]','interpreter','Latex')
ylabel('$\psi$ [$^o$]','interpreter','Latex')
% axis([0 XX(end,end) min(min(XX([6 18],:))*180/pi) max(max(XX([6 18],:))*180/pi)])
axis([0 XX(end,end) -200 200])
grid

set(le1,'Interpreter','latex');
set(le2,'Interpreter','latex');
set(le3,'Interpreter','latex');

%% --------------------------------------------------------------
% Euler angles - rad
figure
subplot(311)
plot(XX(end,:),XX(4,:)',color_d,'LineWidth',line_d),hold on;  % roll
plot(XX(end,:),XX(16,:)',color_r,'LineWidth',line_r)  % roll
le1 = legend('Desired','Real');
xlabel('Time [s]','interpreter','Latex')
ylabel('$\phi$ [rad]','interpreter','Latex')
% axis([0 XX(end,end) min(min(XX([4 16],:))*180/pi) max(max(XX([4 16],:))*180/pi)])
axis([0 XX(end,end) -0.1 0.1])
grid

subplot(312) % pitch
plot(XX(end,:),XX(5,:)',color_d,'LineWidth',line_d),hold on;  
plot(XX(end,:),XX(17,:)',color_r,'LineWidth',line_r)  
le2 = legend('Desired','Real');
xlabel('Time [s]','interpreter','Latex')
ylabel('$\theta$ [rad]','interpreter','Latex')
% axis([0 XX(end,end) min(min(XX([5 17],:))*180/pi) max(max(XX([5 17],:))*180/pi)])
axis([0 XX(end,end) -0.1 0.1])
grid

subplot(313) % yaw
plot(XX(end,:),XX(6,:)',color_d,'LineWidth',line_d),hold on;  
plot(XX(end,:),XX(18,:)',color_r,'LineWidth',line_r)  
le3 = legend('Desired','Real');
xlabel('Time [s]','interpreter','Latex')
ylabel('$\psi$ [rad]','interpreter','Latex')
% axis([0 XX(end,end) min(min(XX([6 18],:))*180/pi) max(max(XX([6 18],:))*180/pi)])
axis([0 XX(end,end) -4 4])
grid

set(le1,'Interpreter','latex');
set(le2,'Interpreter','latex');
set(le3,'Interpreter','latex');




%% --------------------------------------------------------------
% Positions
figure
subplot(311)  % x
plot(XX(end,:),XX(1,:)',color_d,'LineWidth',line_d),hold on;  
plot(XX(end,:),XX(13,:)',color_r,'LineWidth',line_r)  
lp1 = legend('Desired','Real');
xlabel('Time [s]','interpreter','Latex')
ylabel('$x$ [m]','interpreter','Latex')
axis([0 XX(end,end) min(min(XX([1 13],:))) max(max(XX([1 13],:)))])
axis([0 XX(end,end) -2 2])
grid

subplot(312)  % y
plot(XX(end,:),XX(2,:)',color_d,'LineWidth',line_d),hold on;  
plot(XX(end,:),XX(14,:)',color_r,'LineWidth',line_r)  
lp2 = legend('Desired','Real');
xlabel('Time [s]','interpreter','Latex')
ylabel('$y$ [m]','interpreter','Latex')
% axis([0 XX(end,end) min(min(XX([2 14],:))) max(max(XX([2 14],:)))])
axis([0 XX(end,end) -2 2])
grid

subplot(313)   % z
plot(XX(end,:),XX(3,:)',color_d,'LineWidth',line_d),hold on;  
plot(XX(end,:),XX(15,:)',color_r,'LineWidth',line_r)  
lp3 = legend('Desired','Real');
xlabel('Time [s]','interpreter','Latex')
ylabel('$z$ [m]','interpreter','Latex')
% axis([0 XX(end,end) min(min(XX([3 15],:))) max(max(XX([3 15],:)))])
axis([0 XX(end,end) -0 2])
grid

set(lp1,'Interpreter','latex');
set(lp2,'Interpreter','latex');
set(lp3,'Interpreter','latex');

%% ----------------------------------------------------------------------
% Errors

% positions
figure
% subplot(211)  
plot(XX(end,:),Xtil(1,:)','-.','LineWidth',line_e),hold on;  
plot(XX(end,:),Xtil(2,:)','--','LineWidth',line_e)
plot(XX(end,:),Xtil(3,:)','LineWidth',line_e)  
lep1 = legend('$x$','$y$','$z$');
xlabel('Time [s]','interpreter','Latex')
ylabel('Error [m]','interpreter','Latex')
% axis([0 XX(end,end) min(min(Xtil(1:3,:))) max(max(Xtil(1:3,:)))])
axis([0 XX(end,end) -2 2])
grid

% angles - graus
figure;
% subplot(212)  % 
plot(XX(end,:),Xtil(4,:)'*180/pi,'-.','LineWidth',line_e),hold on;  
plot(XX(end,:),Xtil(5,:)'*180/pi,'--','LineWidth',line_e)  
plot(XX(end,:),Xtil(6,:)'*180/pi,'LineWidth',line_e)  
lep2 = legend('$\phi$','$\theta$','$\psi$');
xlabel('Time [s]','interpreter','Latex')
ylabel('Error [$^o$]','interpreter','Latex')
% axis([0 XX(end,end) min(min(Xtil(4:6,:))) max(max(Xtil(4:6,:)))])
axis([0 XX(end,end) -5 5])
grid

% angles - rad
figure
plot(XX(end,:),Xtil(4,:)','-.','LineWidth',line_e),hold on;  
plot(XX(end,:),Xtil(5,:)','--','LineWidth',line_e)  
plot(XX(end,:),Xtil(6,:)','LineWidth',line_e)  
lep3 = legend('$\phi$','$\theta$','$\psi$');
xlabel('Time [s]','interpreter','Latex')
ylabel('Error [rad]','interpreter','Latex')
% axis([0 XX(end,end) min(min(Xtil(4:6,:))) max(max(Xtil(4:6,:)))])
axis([0 XX(end,end) -0.1 0.1])
grid

set(lep1,'Interpreter','latex');
set(lep2,'Interpreter','latex');
set(lep3,'Interpreter','latex');

