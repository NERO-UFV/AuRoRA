%% Rotina para plotar gráfico do controle de posicao do pioneer
clear all
close all
clc

%% Busca pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Carrega dados
data = load('Traj_20180925T160337.txt');
% data = load('PositionMobileSim_20180809T125712.txt');
% data2 = load('Position_20180809T124007.txt');

%% Declara o robo
P = Pioneer3DX;

%% Assign data
% time        = data(:,end);
% 
% P.pPos.Xd   = data(:,(1:12))';       % desired pose
% P.pPos.X    = data(:,12+(1:12))';    % real pose
% P.pSC.Ud    = data(:,24+(1:2))';     % control signal
% P.pSC.U     = data(:,26+(1:2))';     % robot velocities
% P.pPos.Xtil = P.pPos.Xd - P.pPos.X;  % pose error

% rho         = data(:,29);
% alfa        = data(:,30);
% theta       = data(:,31);
% 

% Parameters
scale  = 1;     % robot model plot scale
Pcolor = 'k';   % robot A color


%% Draw results
line_d  = 1;      % desired variables linewidth
line_r  = 0.5;    % real variables linewidth
line_e  = 0.8;    % erros line width
color_d = 'b--';  % line color desired variables
color_r = 'r';    % line color real variables
color_p = 'b';   % line color desired points 


%% Plota graficos
%% Trajectory 2D - superior view

% Caminho
figure
axis equal
plot(data(:,13)',data(:,14)','-.','LineWidth',1),hold on;
plot(data2(:,13)',data2(:,14)','LineWidth',1);
% plot(data(1,13),data(1,14),'ko','MarkerSize', 9,'LineWidth',2);
% plot(data(end,13),data(end,14),'k*','MarkerSize', 9,'LineWidth',2);
xlabel('$x$ [m]','interpreter','Latex')
ylabel('$y$ [m]','interpreter','Latex')
lt1 = legend('Simulado','Experimento','Location','SouthEast');
set(lt1,'Interpreter','latex');
% box on
% axis ([-1 3 -1 3])
grid on
title('Caminho','interpreter','Latex')

% rho
figure
axis equal
plot(data(:,end)',data(:,29)','-.','LineWidth',1),hold on;
plot(data2(:,end)',data2(:,29)','LineWidth',1);
xlabel('Tempo [s]','interpreter','Latex')
ylabel('$\rho$ [m]','interpreter','Latex')
lt1 = legend('Simulado','Experimento','Location','NorthEast');
set(lt1,'Interpreter','latex');
% box on
% axis ([-1 3 -1 3])
grid on


% alfa
figure
axis equal
plot(data(:,end)',data(:,30)','-.','LineWidth',1),hold on;
plot(data2(:,end)',data2(:,30)','LineWidth',1);
xlabel('Tempo [s]','interpreter','Latex')
ylabel('$\alpha$ [rad]','interpreter','Latex')
lt1 = legend('Simulado','Experimento','Location','NorthEast');
set(lt1,'Interpreter','latex');
% box on
% axis ([-1 3 -1 3])
grid on



% % theta
% figure
% axis equal
% plot(data(:,end)',data(:,31)','-.','LineWidth',1),hold on;
% plot(data2(:,end)',data2(:,31)','LineWidth',1);
% xlabel('Tempo [s]','interpreter','Latex')
% ylabel('$\theta$ [rad]','interpreter','Latex')
% lt1 = legend('Simulado','Experimento','Location','NorthEast');
% set(lt1,'Interpreter','latex');
% % box on
% % axis ([-1 3 -1 3])
% grid on

% Velocidades
figure
axis equal
plot(data(:,end)',data(:,27)','-.','LineWidth',1),hold on;
plot(data2(:,end)',data2(:,27)','LineWidth',1);
% plot(data(1,13),data(1,14),'ko','MarkerSize', 9,'LineWidth',2);
% plot(data(end,13),data(end,14),'k*','MarkerSize', 9,'LineWidth',2);
xlabel('Tempo [s]','interpreter','Latex')
ylabel('$v$ [m/s]','interpreter','Latex')
lt1 = legend('Simulado','Experimento','Location','NorthEast');
set(lt1,'Interpreter','latex');
% box on
% axis ([-1 3 -1 3])
grid on

figure
axis equal
plot(data(:,end)',data(:,28)','-.','LineWidth',1),hold on;
plot(data2(:,end)',data2(:,28)','LineWidth',1);
% plot(data(1,13),data(1,14),'ko','MarkerSize', 9,'LineWidth',2);
% plot(data(end,13),data(end,14),'k*','MarkerSize', 9,'LineWidth',2);
xlabel('Tempo [s]','interpreter','Latex')
ylabel('$\omega$ [rad/s]','interpreter','Latex')
lt1 = legend('Simulado','Experimento','Location','NorthEast');
set(lt1,'Interpreter','latex');
% box on
% axis ([-1 3 -1 3])
grid on

