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
% data = load('FL3d_PositionExp20181003T194008.txt');
data = load('FL3d_PositionExp20181011T194510.txt');
%% Atribuição de variáveis
%Tempo da simulação
time  = data(:,end);          % tempo da simulação (s)

% Pioneer data
PXd   = data(:,(1:12))';       % desired pose
PX    = data(:,12+(1:12))';    % real pose
PUd   = data(:,24+(1:2))';     % control signal
PU    = data(:,26+(1:2))';     % velocidades do robô

% Drone data
AXd    = data(:,28+(1:12))';
AX     = data(:,40+(1:12))';
AUd    = data(:,52+(1:4))';
AU     = data(:,56+(1:4))';

% Erros
PXtil  = PXd - PX;              % erro de postura
AXtil  = AXd - AX;

% Formation data
Qd   = data(:,60+(1:6))';   % desired formation
Q    = data(:,66+(1:6))';   % current formation

Qtil = Qd - Q;              % error

% Quadrant correction 
for k=1:length(Qd)
    if abs(Qd(5,k)) > pi
        if Qd(5,k) > 0
            Qd(5,k) = Qd(5,k) - 2*pi;
        else
            Qd(5,k) = Qd(5,k) + 2*pi;
        end
    end
    if abs(Q(5,k)) > pi
        if Q(5,k) > 0
            Q(5,k) = Q(5,k) - 2*pi;
        else
            Q(5,k) = Q(5,k) + 2*pi;
        end
    end
     if abs(Qtil(5,k)) > pi
        if Qtil(5,k) > 0
            Qtil(5,k) = Qtil(5,k) - 2*pi;
        else
            Qtil(5,k) = Qtil(5,k) + 2*pi;
        end
    end
end

% Robots desired positions
for kk = 1:length(Qd)
xf   = Qd(1,kk);  yf    = Qd(2,kk);   zf = Qd(3,kk);
rhof = Qd(4,kk);  alfaf = Qd(5,kk);   betaf = Qd(6,kk);

x2(kk) = xf + rhof*cos(alfaf)*cos(betaf);
y2(kk) = yf + rhof*sin(alfaf)*cos(betaf);
z2(kk) = zf + rhof*sin(betaf);
end
% Angle conversion (rad2deg)
% PX(4:6,:) = rad2deg(PX(4:6,:));
% PU(2,:) = rad2deg(U(2,:));
% PXd(4:6,:) = rad2deg(PXd(4:6,:));
% PUd(2,:) = rad2deg(Ud(2,:));

% AX(4:6,:) = rad2deg(AX(4:6,:));
% AU([1 2 4],:) = rad2deg(AU([1 2 4],:));
% AXd(4:6,:) = rad2deg(AXd(4:6,:));
% AUd([1 2 4],:) = rad2deg(AUd([1 2 4],:));

Qd([5 6],:) = rad2deg(Qd([5 6],:));
Qtil([5 6],:) = rad2deg(Qtil([5 6],:));

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
box on
% ax = axis;
% set(gca,'xticklabel',[0 1])
% set(gca,'yticklabel',[])
% set(gca,'zticklabel',[])
% set(gca,'xticklabel',[])

hold on, grid on;

% Percourse made
p1 = plot3(PX(1,:),PX(2,:),PX(3,:)+h,'r-','LineWidth',sl);
p2 = plot3(AX(1,:),AX(2,:),AX(3,:),'b-','LineWidth',sl);
% title('Posição dos Robôs','fontSize',lt);
xlabel('$x$ [m]','interpreter','Latex'),ylabel('$y$ [m]','Interpreter','latex');
zlabel('$z$ [m]','interpreter','Latex');

% % Desired positions
pd1 = plot3(Qd(1,:),Qd(2,:),PXd(3,:)+h,'ko','LineWidth',sl);hold on % pioneer
pd2 = plot3(x2,y2,z2,'kx','MarkerSize',12,'LineWidth',sl);     % drone


% plot robots and formation lines
for k = 1:step:length(time)
    
    % Pioneer center position
    P.pPos.Xc([1 2 6]) = PX([1 2 6],k) - ...
        [P.pPar.a*cos(PX(6,k)); P.pPar.a*sin(PX(6,k)); 0];
    % ArDrone position
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
%     
%     % plot formation line
%     x = [X(1,k)    AX(1,k)];
%     y = [X(2,k)    AX(2,k)];
%     z = [X(3,k)+h   AX(3,k)];
%       
%     pl = line(x,y,z);
%     pl.Color = 'g';
%     pl.LineStyle = '-';
%     pl.LineWidth = 1;
%     
%     hold on
%     % plot desired formation line
%     xd = [Xd(1,k)    AXd(1,k)];
%     yd = [Xd(2,k)    AXd(2,k)];
%     zd = [Xd(3,k)+h  AXd(3,k)];
%     
%     pld = line(xd,yd,zd);
%     pld.Color = 'm';
%     pld.LineStyle = '--';
    pause(1/30)
end
%
lg1 = legend([p1 p2 pl pld],{'Rob\^{o} 1', 'Rob\^{o} 2','Forma\c{c}\~ao real','Forma\c{c}\~ao desejada'});
% % lg1 = legend([pld pl],{'Formation line'});
% %
% lg1.FontSize = 11;
% lg1.Location = 'SouthEast';
% set(lg1,'Interpreter','latex');
% % legend('boxoff')
%
% Velocidades ...............................................................
% Pioneer
figure;
subplot(211), plot(time,PU(1,:),'LineWidth',sl);
% legend({'u'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$u$ [m/s]','interpreter','Latex');
title('(a)','Interpreter','latex');
subplot(212), plot(time,PU(2,:),'LineWidth',sl);
% legend({'$\omega$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\omega$ [rad/s]','interpreter','Latex');
title('(b)','Interpreter','latex');

% ArDrone
figure;
subplot(221),plot(time,AUd(1,:),'--','LineWidth',sl);hold on;
plot(time,AU(1,:),'LineWidth',sl);
legend({'$\phi_d$','$\phi_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\phi$ [rad]','interpreter','Latex');
title('(a)','Interpreter','latex');

subplot(222),plot(time,AUd(2,:),'--','LineWidth',sl);hold on;
plot(time,AU(2,:),'LineWidth',sl);
legend({'$\theta_d$','$\theta_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\theta$ [rad]','interpreter','Latex');
title('(b)','Interpreter','latex');

subplot(223),plot(time,AUd(3,:),'--','LineWidth',sl);hold on;
plot(time,AU(3,:),'LineWidth',sl);
legend({'$\dot{z}_{d}$','$\dot{z}_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]),ylim([-1 1]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\dot{z}$ [m/s]','interpreter','Latex');
title('(c)','Interpreter','latex');

subplot(224),plot(time,AUd(4,:),'--','LineWidth',sl);hold on;
plot(time,AU(4,:),'LineWidth',sl);
legend({'$\dot{\psi}_{d}$','$\dot{\psi}_{r}$'},'Interpreter','latex');
grid on,xlim([0 time(end)]),ylim([-1 1]);
xlabel('Time [s]','interpreter','Latex'),ylabel('$\dot{\psi}$ [rad/s]','interpreter','Latex');
title('(d)','Interpreter','latex');


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



