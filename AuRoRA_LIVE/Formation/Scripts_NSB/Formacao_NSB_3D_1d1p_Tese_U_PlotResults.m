

%% Controle de Formação Baseado em Espaço Nulo
% Analise de dados usando Bebop - Tese 2019
% Mauro Sérgio Mafra

% Resetar 
clear all;   
close all;
warning off; 
clc;

%% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

try
    fclose(instrfindall);
end

%% Load Class
try     
    % Load Classes
    A = ArDrone;
    disp('################### Load Class Success #######################');            
catch ME
    disp(' ');
    disp(' ################### Load Class Issues #######################');
    disp(' ');
    disp(' ');
    disp(ME);
    
    return;
end

% Files to be loaded
fileName(1) = "20191106_1932_NSB_PF_K1[0.2 0.2 0.2 0.2 0.2 0.2] K2 [0.4 0.4 0.4 0.4 0.4].mat";
fileName(2) = "20191106_1932_NSB_PP_K1[0.2 0.2 0.2 0.2 0.2 0.2] K2 [0.4 0.4 0.4 0.4 0.4].mat";
fileName(3) = "20191106_1932_NSB_SP_K1[0.2 0.2 0.2 0.2 0.2 0.2] K2 [0.4 0.4 0.4 0.4 0.4].mat";

fileName(4) = "20191108_1435_NSB_PF_K1_0.4[2 2 2 2 4 7] K2 [1 1 1 1 1].mat";
fileName(5) = "20191108_1435_NSB_SP_K1_0.4[2 2 2 2 4 7] K2 [1 1 1 1 1].mat";
fileName(6) = "20191108_1442_NSB_PF_K1_0.4[2 2 2 2 4 7] K2 [1 1 1 1 1].mat";
fileName(7) = "20191108_1444_NSB_PP_K1_0.4[2 2 2 2 4 7] K2 [1 1 1 1 1].mat";

fileName(8)  = "20191108_1446_NSB_SP_K1_0.4[2 2 2 2 4 7] K2 [1 1 1 1 1]_60s.mat";
fileName(9)  = "20191108_1450_NSB_PF_K1_0.4[2 2 2 2 4 7] K2 [1 1 1 1 1]_60s.mat";
fileName(10)  = "20191108_1453_NSB_PP_K1_0.4[2 2 2 2 4 7] K2 [1 1 1 1 1]_60s.mat";

fileName(13) = "20191108_1613_NSB_PF_K1[0.8 0.8 0.8 2 4 7] K2 [1 1 1 1 1]_60s.mat";
fileName(14) = "20191108_1618_NSB_PP_K1[0.8 0.8 0.8 2 4 7] K2 [1 1 1 1 1]_60s.mat";
fileName(15) = "20191108_1634_NSB_SP_K1[0.8 0.8 0.8 2 4 7] K2 [1 1 1 1 1]_60s.mat";

fileName(16) = "20191108_1852_NSB_SP_K1[0.8 0.8 0.8 2 4 7] K2 [1 1 1 1 1]_60s_Posicao_Pouso.mat";
fileName(17) = "20191108_1925_NSB_PF_K1[0.8 0.8 0.8 2 4 7] K2 [1 1 1 1 1]_60s_Posicao_Pouso.mat";



% Change files to be Loaded
index = 17;
load(fileName(index));



%% PLOTA RESULTADOS
sl= 1; %'default';    % largura da linha
st = 14;  % tamanho da fonte
ss = 2;   % tamanho dos símbolos

% Posição dos robôs
% Parameters
scale  = 1;     % robot model plot scale
Pcolor = 'k';   % robot A color
step   = 200;   % model plot step
h      = 0.1;   % line height
iFigTitle = "Figure"; 


%% Atribuição de variáveis
%Tempo da simulação
time  = data(:,end);                % tempo (s)

% Pioneer data
PXd   = data(:,(1:12))';       % desired pose
PX    = data(:,12+(1:12))';    % real pose
PUd   = data(:,24+(1:2))';     % control signal
PU    = data(:,26+(1:2))';     % velocidades do robô
PXtil = PXd - PX; % erro de postura

% Drone data
AXd   = data(:,28+(1:12))';
AX    = data(:,40+(1:12))';
AUd   = data(:,52+(1:4))';
AU    = data(:,56+(1:4))';
AXtil = AXd - AX;

% Formation data
Qd   = data(:,60+(1:6))';   % desired formation
Qtil = data(:,66+(1:6))';   % formation error

 
% Robots desired positions
for kk = 1:length(Qd)
    xf   = Qd(1,kk);  yf    = Qd(2,kk);   zf = Qd(3,kk);
    rhof = Qd(4,kk);  alfaf = Qd(5,kk);   betaf = Qd(6,kk);

    x2(kk) = xf + rhof*cos(alfaf)*cos(betaf);
    y2(kk) = yf + rhof*sin(alfaf)*cos(betaf);
    z2(kk) = zf + rhof*sin(betaf);
end



% Prepare enviroment


%% Plot results
%figure;
hold on;     
    
    axis equal
    axis ([-2 2 -2.5 2.5 0 2.5])
    set(gca,'Box','on')

    % Percourse made
    fig1 = plot3(data(:,13),data(:,14),data(:,15),'r','LineWidth',0.8);
    fig2 = plot3(data(:,41),data(:,42),data(:,43),'b','LineWidth',0.8);

    % Initial positions
    ps1 = plot3(PX(1,1),PX(2,1),PX(3,1)+h,'k^','MarkerSize',5,'LineWidth',3);
    ps2 = plot3(AX(1,1),AX(2,1),AX(3,1),'k^','MarkerSize',5,'LineWidth',3);

    % % Desired positions
    %pd1 = plot3(Qd(1,:),Qd(2,:),PXd(3,:)+h,'k.','MarkerSize',20,'LineWidth',2);hold on % pioneer
    %pd2 = plot3(x2,y2,z2,'k.','MarkerSize',20,'LineWidth',2);     % drone

    % lg1 = legend([pl ps1 pd1],{'Formation Line','Start position','Desired Positions'});
    %lg1 = legend([ps1 pd1],{'Start positions','Desired Positions'});
    lg1 = legend([ps1],{'Posição Inicial'});
    %lg1 = legend(pl,{'Formation line'});
    %
    lg1.FontSize = 11;
    lg1.Location = 'northEast';
    set(lg1,'Interpreter','latex');
    %legend('boxoff')

    xlabel('$x$ [m]','interpreter','Latex'),
    ylabel('$y$ [m]','Interpreter','latex');
    zlabel('$z$ [m]','interpreter','Latex');

    title('Controlador Convencional')
    %title('NSB Prioridade de Forma')
    %title('NSB Prioridade de Posição')
    grid on;

hold off;




% % Erro Posição Formação
Xtil = data(:,end-6:end-1);
figure();
hold on;
    grid on;
    plot(data(:,end),Xtil(:,1));
    plot(data(:,end),Xtil(:,2));
    plot(data(:,end),Xtil(:,3));
    plot(data(:,end),Xtil(:,4));
    plot(data(:,end),Xtil(:,5));
    plot(data(:,end),Xtil(:,6));
    %title('Erro de Fromação Controlador Convencional');
    title('Erro de Fromação NSB Prioridade de Forma');
    %title('Erro de Fromação NSB Prioridade de Posição');
    legend('Pos X','Pos Y','Pos Z', 'Rho', 'Apha', 'Beta');
    xlabel('Tempo[s]');
    ylabel('Erro [m]');
hold off;






% % Velocidades ...............................................................
% % Pioneer
% figure;
% subplot(121), plot(time,PUd(1,:),'--','LineWidth',sl), hold on;
% plot(time,PU(1,:),'LineWidth',sl);
% legend({'$u_{d}$','$u_{r}$'},'Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Time [s]','interpreter','Latex'),ylabel('Velocity [m/s]','interpreter','Latex');
% title('(a)','Interpreter','latex');
% subplot(122), plot(time,PUd(2,:),'--','LineWidth',sl), hold on;
% plot(time,PU(2,:),'LineWidth',sl)
% legend({'$\omega_{d}$','$\omega_{r}$'},'Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Time [s]','interpreter','Latex'),ylabel('Velocity [rad/s]','interpreter','Latex');
% title('(b)','Interpreter','latex');
% set(gcf, 'name', iFigTitle);
% 
% 
% 
% % ArDrone
% figure;
% 
% subplot(221),plot(time,AUd(1,:),'--','LineWidth',sl);hold on;
% plot(time,AU(1,:),'LineWidth',sl);
% legend({'$\phi_{d}$','$\phi_{r}$'},'Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Time [s]','interpreter','Latex'),ylabel('$\phi$ [rad]','interpreter','Latex');
% title('(a)','Interpreter','latex');
% set(gcf, 'name', iFigTitle);
% 
% subplot(222),plot(time,AUd(2,:),'--','LineWidth',sl);hold on;
% plot(time,AU(2,:),'LineWidth',sl);
% legend({'$\theta_{d}$','$\theta_{r}$'},'Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Time [s]','interpreter','Latex'),ylabel('$\theta$ [rad]','interpreter','Latex');
% title('(b)','Interpreter','latex');
% 
% subplot(223),plot(time,AUd(3,:),'--','LineWidth',sl);hold on;
% plot(time,AU(3,:),'LineWidth',sl);
% legend({'$\dot{z}_{d}$','$\dot{z}_{r}$'},'Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Time [s]','interpreter','Latex'),ylabel('$\dot{z}$ [m/s]','interpreter','Latex');
% title('(c)','Interpreter','latex');
% 
% subplot(224),plot(time,AUd(4,:),'--','LineWidth',sl);hold on;
% plot(time,AU(4,:),'LineWidth',sl);
% legend({'$\dot{\psi}_{d}$','$\dot{\psi}_{r}$'},'Interpreter','latex');
% grid on,xlim([0 time(end)]);
% xlabel('Time [s]','interpreter','Latex'),ylabel('$\dot{\psi}$ [rad/s]','interpreter','Latex');
% title('(d)','Interpreter','latex');
% 
% 
% 
% 
% %
% % Erro da formação ....................................................................
% figure;
% % Position
% subplot(211),plot(time,Qtil(1,:),'b--','LineWidth',sl),hold on;
% plot(time,Qtil(2,:),'r-','LineWidth',sl);
% plot(time,Qtil(4,:),'g-.','LineWidth',.5);
% xlabel('Time [s]','interpreter','Latex'),ylabel('Error [m]','interpreter','Latex');
% xlim([0 time(end)]);
% title('(a)','Interpreter','latex');
% set(gcf, 'name', iFigTitle);
% 
% lg3 = legend('$x_f$','$y_f$','$\rho_f$');
% lg3.FontSize = 10;
% lg3.Location = 'NorthEast';
% set(lg3,'Interpreter','latex');
% grid on;
% 
% 
% % Angles
% subplot(212),plot(time,Qtil(5,:),'b--','LineWidth',sl),hold on;
% plot(time,Qtil(6,:),'r-','LineWidth',sl);
% xlabel('Time [s]','interpreter','Latex'),ylabel('Error [rad]','interpreter','Latex');
% xlim([0 time(end)]);
% title('(b)','Interpreter','latex');
% 
% % lg3 = legend('$x_f [m]$','$y_f [m]$','$z_f [m]$','$\rho_f [m]$','$\alpha_f [rad]$','$\beta_f [rad]$');
% lg4 = legend('$\alpha_f$','$\beta_f$');
% lg4.FontSize = 10;
% lg4.Location = 'NorthEast';
% set(lg4,'Interpreter','latex');
% grid on
% %
% 


disp("Figures Printed...");

  