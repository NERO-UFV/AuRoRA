%% Rotina para plotar gráficos de Formacaolinha3D
clear
close all
clc

%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Declara os robos
P  = Pioneer3DX;
A  = ArDrone;
LF = LineFormation3D;

%% Carrega dados
% data = load('FL3d_TrajectoryUtimateBebop20200114T012033.txt');
data = load('FL3d_TrajectoryUtimateBebop20200113T232638.txt');
%% Atribuição de variáveis
%Tempo da simulação
time  = data(:,end);                % tempo (s)

% Pioneer data
PXd   = data(:,(1:12))';       % desired pose
PX    = data(:,12+(1:12))';    % real pose
PUd   = data(:,24+(1:2))';     % control signal
% PU    = data(:,26+(1:2))';     % velocidades do robô
PXtil = PXd - PX; % erro de postura

% Drone data
AXd   = data(:,28+(1:12))';
AX    = data(:,40+(1:12))';
AUd   = data(:,52+(1:6))';
% AU    = data(:,58+(1:4))';


% Formation data
Qd   = data(:,62+(1:6))';   % desired formation
Q    = data(:,68+(1:6))';   % real formation

% % Alpha quadrant adjust
% a = 1;
% for ii = 1:length(Q)-1
%     angle = Q(5,ii+1) - Q(5,ii);
%     if abs(angle)<6.5  && abs(angle) > 6  % detect breaks
%         ind(a) = ii;   % save break vector indice
%         a = a+1;
%     end
% end      % detect breaks
%
% for f = 1:2:length(ind)-1
%     for jj = ind(f):ind(f+1)-1
%         if Q(5,jj+1)>0      % -pi --> +pi
%             Q(5,jj+1) = Q(5,jj+1) - 2*pi;
%         elseif Q(5,jj+1)<0  % +pi --> -pi
%             Q(5,jj+1) = Q(5,jj+1) + 2*pi;
%         end
%     end
% end  % do the thing
%
Qtil = Qd - Q;              % formation error


% Second robot desired positions
for kk = 1:length(Qd)
    xf   = Qd(1,kk);  yf    = Qd(2,kk);   zf = Qd(3,kk);
    rhof = Qd(4,kk);  alfaf = Qd(5,kk);   betaf = Qd(6,kk);
    
    x2(kk) = xf + rhof*cos(alfaf)*cos(betaf);
    y2(kk) = yf + rhof*sin(alfaf)*cos(betaf);
    z2(kk) = zf + rhof*sin(betaf);
end

AXd(1:3,:) = [x2;y2;z2];
% Position Error
AXtil = AXd - AX;

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
% Tamanho das letras, gráficos e símbolos
sizeDesired = 2;      % largura das linhas de valores desejados
sizeReal    = 1;      % largura da linha de valor real
sizeLabel   = 14;     % tamanho da fonte dos labels
sizeSymbol  = [20 2];  % [tamanho dos símbolos   espessura da linha]
sizeLegenda = 15;     % tamanho das legendas
location    = 'SouthEast'; %'NorthEast';
% Posição dos robôs
% Parameters
scale  = 1;     % pioneer model plot scale
Pcolor = 'k';   % pioneer
step   = 10;   % model plot step
h      = 0.1;   % line height (to plot above the ground)

% figure(1);
% axis equal
% axis ([0 50 -1 1])
% ylim ([-1 1]);
% set(gca,'Box','on')

% pause(3);

% ax = axis;
% set(gca,'xticklabel',[0 1])
% set(gca,'yticklabel',[])
% set(gca,'zticklabel',[])
% set(gca,'xticklabel',[])
% pause    % para dar tempo de iniciar captura de tela
% hold on, grid on;

% xlabel('$Time$ [s]','interpreter','Latex'),ylabel('$Error$ [m]','Interpreter','latex');

%% One window

% pause(5);
% delete(lg1);
% legend off
% errX = plot(time(1),Qtil(1,1),'r-','LineWidth',1); hold on;
% %
% % for k = 1:step:length(time)
% %
% %
% %     % delete previous points
% % %      delete (errX);
% %     %     % Errors
% %    errX = plot(time(1:k),Qtil(1,1:k),'r-','LineWidth',2);
% %    errY = plot(time(1:k),Qtil(2,1:k),'g-','LineWidth',2);
% %    errRho = plot(time(1:k),Qtil(4,1:k),'b-','LineWidth',2);
% %    errBeta = plot(time(1:k),Qtil(6,1:k),'k-','LineWidth',2);
% % %    drawnow
% %
% %     pause(.1)
% %
% % end

%% Subplot
figure, hold on;
title('Formation Errors','Interpreter','latex','FontSize',15)
% axis([0 50 -1 1])
subplot(411),ylabel('$X$ [m]','Interpreter','latex','FontSize',sizeLabel),axis([0 50 -1 1]), grid on;
subplot(412),ylabel('$Y$ [m]','Interpreter','latex','FontSize',sizeLabel),axis([0 50 -1 1]), grid on;
subplot(413),ylabel('$\rho$ [m]','Interpreter','latex','FontSize',sizeLabel),axis([0 50 -1 1]), grid on;
subplot(414),xlabel('$Time$ [s]','interpreter','Latex','FontSize',sizeLabel)
ylabel('$\beta$ [rad]','Interpreter','latex'),axis([0 50 -1 1]), grid on;

pause(7)

for k = 1:step:length(time)
    
   
    % x error
    subplot(411);
    errX = plot(time(1:k),Qtil(1,1:k),'r-','LineWidth',2);
    ylabel('$X$ [m]','Interpreter','latex','FontSize',sizeLabel),axis([0 50 -1 1]);
    grid on
    % y error
    subplot(412);
    errY = plot(time(1:k),Qtil(2,1:k),'g-','LineWidth',2);
    ylabel('$Y$ [m]','Interpreter','latex','FontSize',sizeLabel),axis([0 50 -1 1]);
    grid on
    % rho error
    subplot(413);
    errRho = plot(time(1:k),Qtil(4,1:k),'b-','LineWidth',2);
    ylabel('$\rho$ [m]','Interpreter','latex','FontSize',sizeLabel+3),axis([0 50 -1 1]);
    grid on
    % beta error
    subplot(414);
    errBeta = plot(time(1:k),Qtil(6,1:k),'k-','LineWidth',2);
    xlabel('$Time$ [s]','interpreter','Latex','FontSize',sizeLabel)
    ylabel('$\beta$ [rad]','Interpreter','Latex','FontSize',sizeLabel+3),axis([0 50 -1 1]);
    grid on
    
    pause(.1)
    
end

% close
