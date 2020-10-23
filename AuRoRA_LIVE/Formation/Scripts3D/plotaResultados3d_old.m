%% Rotina para plotar gráficos de Formacaolinha3D
clear all
close all
clc

%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Declara os robos
P = Pioneer3DX;
A = ArDrone;
LF = LineFormation3D;

%% Carrega dados
data = load('FL3dSIMU_20180925T213855.txt');

%% Atribuição de variáveis
%Tempo da simulação
time  = data(:,end);                % tempo (s)

% Pioneer data
Xd   = data(:,(1:12))';       % desired pose
X    = data(:,12+(1:12))';    % real pose
Ud   = data(:,24+(1:2))';     % control signal
U    = data(:,26+(1:2))';     % velocidades do robô
Xtil = Xd - X; % erro de postura

% Drone data
AXd   = data(:,28+(1:12))';
AX    = data(:,40+(1:12))';
AUd    = data(:,52+(1:4))';
AU     = data(:,56+(1:4))';
AXtil = AXd - AX;

% Formation data
Qd = data(:,60+(1:6))';   % desired formation
Qtil = data(:,66+(1:6))';
xf = Qd(1,1);yf = Qd(2,1);zf = Qd(3,1);
rhof = Qd(4,1);alfaf= Qd(5,1);betaf = Qd(6,1);
x2 = xf + rhof*cos(alfaf)*cos(betaf);
y2 = yf + rhof*sin(alfaf)*cos(betaf);
z2 = zf + rhof*sin(betaf);

%% Angle convertion (rad2deg)
% X(4:6,:) = rad2deg(X(4:6,:));
% U(2,:) = rad2deg(U(2,:));
% Xd(4:6,:) = rad2deg(Xd(4:6,:));
% Ud(2,:) = rad2deg(Ud(2,:));

% AX(4:6,:) = rad2deg(AX(4:6,:));
% AU([1 2 4],:) = rad2deg(AU([1 2 4],:));
% AXd(4:6,:) = rad2deg(AXd(4:6,:));
% AUd([1 2 4],:) = rad2deg(AUd([1 2 4],:));

% Qd([5 6],:) = rad2deg(Qd([5 6],:));
% Qtil([5 6],:) = rad2deg(Qtil([5 6],:));

%% PLOTA RESULTADOS
sl= 1; %'default';    % largura da linha
st = 14;  % tamanho da fonte
ss = 2;   % tamanho dos símbolos

% Posição dos robôs
% Parameters
scale  = 1;     % robot model plot scale
Pcolor = 'k';   % robot A color
step   = 80;   % model plot step
h      = 0.1;   % line height

figure(1);
axis equal
% ax = axis;
% set(gca,'xticklabel',[0 1])
% set(gca,'yticklabel',[])
% set(gca,'zticklabel',[])
% set(gca,'xticklabel',[])

hold on, grid on;

% Percourse made
p1 = plot3(X(1,:),X(2,:),X(3,:)+h,'r-','LineWidth',sl);
p2 = plot3(AX(1,:),AX(2,:),AX(3,:),'b-','LineWidth',sl);
% title('Posição dos Robôs','fontSize',lt);
xlabel('$x$ [m]','interpreter','Latex'),ylabel('$y$ [m]','Interpreter','latex');
zlabel('$z$ [m]','interpreter','Latex');

% % Desired Percourse
% pd1 = plot3(Xd(:,1),Xd(:,2),Xd(:,3)+h,'r--','LineWidth',sl);hold on
% pd2 = plot3(AXd(:,1),AXd(:,2),AXd(:,3)+h,'b--','LineWidth',sl);
% 
% 
% Inital/Final pose
plot3(X(1,1),X(2,1), X(3,1)+h, 'ro','LineWidth',ss); % início
hold on
plot3(AX(1,1),AX(2,1), AX(3,1),'bo','LineWidth',ss); % inicio
plot3(Qd(1),Qd(2), Qd(3),'rx','LineWidth',ss); % chegada
plot3(x2,y2,z2,'bx','LineWidth',ss);  % chegada


% plot robots and formation lines
for k = 1:step:length(time)
    
    Xc = X(:,k);
    A.pPos.X = AX(:,k);
    
%     % plota trianglinho
%     P.mCADplot2D('r');
%     A.mCADplot2D('b');

%   % Plota pioneer3dx bonitão 
    P.mCADdel;
    P.mCADplot(scale,Pcolor);
    A.mCADplot;
     
    drawnow
    view(45,30)
    % plot formation line 
    x = [X(1,k)    AX(1,k)];
    y = [X(2,k)    AX(2,k)];
    z = [X(3,k)+h   AX(3,k)];
    
%  
%     x = [Xc(1)   Ar.pPos.X(1)];
%     y = [Xc(2)   Ar.pPos.X(2)];
%     z = [Xc(3)  Ar.pPos.X(3)];
%        
    pl = line(x,y,z);
    pl.Color = 'g';
    pl.LineStyle = '-';
    pl.LineWidth = 1;
    
    hold on
    % plot desired formation line
    xd = [Xd(1,k)    AXd(1,k)];
    yd = [Xd(2,k)    AXd(2,k)];
    zd = [Xd(3,k)+h  AXd(3,k)];
   
    pld = line(xd,yd,zd);
    pld.Color = 'm';
    pld.LineStyle = '--';
    
end
% 
% lg1 = legend([p1 p2 pl pld],{'Robo 1', 'Robo 2','Formacao real','Formacao desejada'});
lg1 = legend([pld pl],{'Desired Formation','Real Formation'});
% 
% lg1.FontSize = 11;
% lg1.Location = 'SouthEast';
set(lg1,'Interpreter','latex');
% % legend('boxoff')
% 
% % Salva a imagem
% saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\posicao.png');

% Velocidades ...............................................................
% Pioneer
figure;
subplot(321), plot(time,U(1,:),'LineWidth',sl);
% legend({'u'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$u$ [m/s]','interpreter','Latex');
title('(a)','Interpreter','latex');
subplot(322), plot(time,U(2,:),'LineWidth',sl);
% legend({'$\omega$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\omega$ [rad/s]','interpreter','Latex');
title('(b)','Interpreter','latex');

% ArDrone
subplot(323),plot(time,AU(1,:),'LineWidth',sl);
% legend({'$\phi$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\phi$ [rad]','interpreter','Latex');
title('(c)','Interpreter','latex');

subplot(324),plot(time,AU(2,:),'LineWidth',sl);
% legend({'$\theta$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\theta$ [rad]','interpreter','Latex');
title('(d)','Interpreter','latex');

subplot(325),plot(time,AU(3,:),'LineWidth',sl);
% legend({'$\dot{z}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\dot{z}$ [m/s]','interpreter','Latex');
title('(e)','Interpreter','latex');

subplot(326),plot(time,AU(4,:),'LineWidth',sl);
% legend({'$\dot{\psi}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\dot{\psi}$ [rad/s]','interpreter','Latex');
title('(f)','Interpreter','latex');


% % % Salva a imagem
% % saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\velLinear.png');
% 
% 
% % % Sinal de controle - velocidade linear
% % figure;
% % hold on, grid on;
% % plot(time,Ud(:,1),'LineWidth',sl);
% % plot(time,AUd(:,1),'LineWidth',sl);
% % title('Sinal de controle (Velocidade Linear)','fontSize',st);
% % xlabel('$Tempo$ [s]','interpreter','Latex'),ylabel('$Velocidade$ [m/s]','interpreter','Latex');
% % legend('Robô 1', 'Robô 2');
% 
% 
% % Velocidades Angulares
% figure;
% hold on, grid on;
% plot(time,U(:,2),'r','LineWidth',sl);
% plot(time,AU(:,2),'b','LineWidth',sl);
% % title('Velocidade Angular','fontSize',st);
% xlabel('$Time$ [s]','interpreter','Latex'),ylabel('$Speed$ [rad/s]','interpreter','Latex');
% legend({'Robot 1', 'Robot 2'},'Interpreter','latex');
% xlim([0 time(end)]);
% 
% % % Salva a imagem
% % saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\velAngular.png');
% 
% 
% % % Sinal de controle - velocidade angular
% % figure;
% % hold on, grid on;
% % plot(time,Ud(:,2),'LineWidth',sl);
% % plot(time,AUd(:,2),'LineWidth',sl);
% % title('Sinal de controle (Velocidade Angular)','fontSize',st);
% % xlabel('$Tempo$ [s]','interpreter','Latex'),ylabel('$Velocidade$ [rad/s]','interpreter','Latex');
% % legend('Robô 1', 'Robô 2');
% 
% %% Erros
% % % Erro de posição
% % figure;
% % hold on, grid on;
% % plot(time,Xtil(:,[1 2]),'LineWidth',sl);
% % plot(time,AXtil(:,[1 2]),'LineWidth',sl);
% % % title('Erro de posição','fontSize',st);
% % xlabel('$Time$ [s]','interpreter','Latex'),ylabel('Error [m]','interpreter','Latex');
% % xlim([0 time(end)]);
% %
% % lg2 = legend('$x_{robot 1}$','$y_{robot 1}$', '$x_{robot 2}$','$y_{robot 2}$');
% % lg2.FontSize = 12;
% % lg2.Location = 'SouthEast';
% % set(lg2,'Interpreter','latex');
% %
% % % % Salva a imagem
% % % saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\ErroPosicao.png');
% 
% 
% Erro da formação ....................................................................
figure;
% Position
subplot(211),plot(time,Qtil(1,:),'b--','LineWidth',sl),hold on;
plot(time,Qtil(2,:),'r-','LineWidth',sl);
plot(time,Qtil(4,:),'g-.','LineWidth',.5);
xlabel('Time [s]','interpreter','Latex'),ylabel('Error [m]','interpreter','Latex');
xlim([0 time(end)]);
title('(a)','Interpreter','latex');

lg3 = legend('$x_f$','$y_f$','$\rho_f$');
lg3.FontSize = 10;
lg3.Location = 'NorthEast';
set(lg3,'Interpreter','latex');
grid on;


% Angles
subplot(212),plot(time,Qtil(5,:),'b--','LineWidth',sl),hold on;
plot(time,Qtil(6,:),'r-','LineWidth',sl);
xlabel('Time [s]','interpreter','Latex'),ylabel('Error [$^{o}$]','interpreter','Latex');
xlim([0 time(end)]);
title('(b)','Interpreter','latex');

% lg3 = legend('$x_f [m]$','$y_f [m]$','$z_f [m]$','$\rho_f [m]$','$\alpha_f [rad]$','$\beta_f [rad]$');
lg4 = legend('$\alpha_f$','$\beta_f$');
lg4.FontSize = 10;
lg4.Location = 'NorthEast';
set(lg4,'Interpreter','latex');
grid on
% 
% % % Salva a imagem
% % saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\Erroformacao.png');
% 
% 
