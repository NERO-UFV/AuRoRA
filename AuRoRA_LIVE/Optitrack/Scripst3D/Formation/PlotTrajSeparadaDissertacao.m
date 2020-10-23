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
% data = load('FL3d_TrajLemni&Elipse20181011T183418.txt');
% data = load('FL3d_TrajElipseUltimate20181011T183738');
data = load('FL3d_TrajElipseUltimate20181011T183738.txt');

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
Q    = data(:,66+(1:6))';   % real formation

Q(4,:) = Q(4,:) ;%+ 0.3;      % add pioneer height at rho distance

% Alpha quadrant adjust
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

Qtil = Qd - Q;              % formation error

% Robots desired positions
for kk = 1:length(Qd)
    xf   = Qd(1,kk);  yf    = Qd(2,kk);   zf = Qd(3,kk);
    rhof = Qd(4,kk);  alfaf = Qd(5,kk);   betaf = Qd(6,kk);
    
    x2(kk) = xf + rhof*cos(alfaf)*cos(betaf);
    y2(kk) = yf + rhof*sin(alfaf)*cos(betaf);
    z2(kk) = zf + rhof*sin(betaf);
end

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

%% PARÂMETROS
% Tamanho das letras, gráficos e símbolos
sizeDesired = 1.2;     % largura das linhas de valores desejados
sizeReal    = 1;       % largura da linha de valor real
sizeLabel   = 18;      % tamanho da fonte dos labels
sizeTitle   = 16;      % tamanho dos títulos
sizeSymbol  = [10 2];  % [tamanho dos símbolos   espessura da linha]
sizeLegenda = 17;      % tamanho das legendas
location    = 'SouthEast'; %'NorthEast';

% Parâmatros dos robôs
scale  = 1;     % pioneer model plot scale
Pcolor = 'k';   % pioneer
step   = 200;   % model plot step
h      = 0.1;   % line height (to plot above the ground)


%% Separate trajectory in 3 parts
% Total de 3 voltas, a ideia é plotar cada volta separadamente
totalLap  = 3;       % total de voltas 
pedacim   = 4;       % total de "partes" para separar. Cada volta tem 4 partes
lap = floor(pedacim/(4*totalLap)*length(AX));
% :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
% primeira volta 
figure;
% subplot(2,3,1)
% percurso desejado
pd1 = plot3(PXd(1,1:lap),PXd(2,1:lap),PXd(3,1:lap)+h,'r--','LineWidth',sizeReal); hold on;
pd2 = plot3(AXd(1,1:lap),AXd(2,1:lap),AXd(3,1:lap),'b--','LineWidth',sizeReal);

% percurso realizado
pr1 = plot3(PX(1,1:lap),PX(2,1:lap),PX(3,1:lap)+h,'r-','LineWidth',sizeReal); hold on;
pr2 = plot3(AX(1,1:lap),AX(2,1:lap),AX(3,1:lap),'b-','LineWidth',sizeReal);

% Ponto final do trecho
plot3(PX(1,lap),PX(2,lap),PX(3,lap)+h,'ksq','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2)); hold on;
plot3(AX(1,lap),AX(2,lap),AX(3,lap),'ksq','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));

% Initial positions
pi1 = plot3(PX(1,1),PX(2,1),PX(3,1)+h,'k^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
pi2 = plot3(AX(1,1),AX(2,1),AX(3,1),'k^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));

% title('Primeira volta','FontSize',sizeTitle,'interpreter','Latex');
xlabel('$x$ [m]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('$y$ [m]','FontSize',sizeLabel,'Interpreter','latex');
zlabel('$z$ [m]','FontSize',sizeLabel,'interpreter','Latex');
axis([-2 2 -2 2 0 1.5])
grid on
box on

% leg11 = legend([ pi1 pd1 pd2 pr1 pr2],{'Posi\c{c}\~{a}o inicial','Trajet\''{o}ria desejada Pioneer 3-DX', ...
%     'Trajet\''{o}ria desejada AR.Drone 2.0','Trajet\''{o}ria Pioneer 3-DX','Trajet\''{o}ria AR.Drone 2.0'});
% leg11.FontSize = sizeLegenda;
% leg11.Location = location;
% set(leg11,'Interpreter','latex');

%::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
% segunda volta 
figure
% subplot(2,3,2)
% percurso desejado
plot3(PXd(1,lap:2*lap),PXd(2,lap:2*lap),PXd(3,lap:2*lap)+h,'r--','LineWidth',sizeReal); hold on;
plot3(AXd(1,lap:2*lap),AXd(2,lap:2*lap),AXd(3,lap:2*lap),'b--','LineWidth',sizeReal);

% percurso realizado
plot3(PX(1,lap:2*lap),PX(2,lap:2*lap),PX(3,lap:2*lap)+h,'r-','LineWidth',sizeReal); hold on;
plot3(AX(1,lap:2*lap),AX(2,lap:2*lap),AX(3,lap:2*lap),'b-','LineWidth',sizeReal);

% Ponto inicial do trecho
plot3(PX(1,lap),PX(2,lap),PX(3,lap)+h,'k^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2)); hold on;
plot3(AX(1,lap),AX(2,lap),AX(3,lap),'k^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));

% Ponto final do trecho
plot3(PX(1,2*lap),PX(2,2*lap),PX(3,2*lap)+h,'ksq','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2)); hold on;
plot3(AX(1,2*lap),AX(2,2*lap),AX(3,2*lap),'ksq','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));

% title('Segunda volta','FontSize',sizeTitle,'interpreter','Latex');
xlabel('$x$ [m]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('$y$ [m]','FontSize',sizeLabel,'Interpreter','latex');
zlabel('$z$ [m]','FontSize',sizeLabel,'interpreter','Latex');
axis([-2 2 -2 2 0 1.5])
grid on
box on

% ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
% terceira volta 
figure
% subplot(2,3,3)
% percurso desejado
ptd1 = plot3(PXd(1,2*lap:end),PXd(2,2*lap:end),PXd(3,2*lap:end)+h,'r--','LineWidth',sizeReal); hold on;
ptd2 = plot3(AXd(1,2*lap:end),AXd(2,2*lap:end),AXd(3,2*lap:end),'b--','LineWidth',sizeReal);

% percurso realizado
ptr1 = plot3(PX(1,2*lap:end),PX(2,2*lap:end),PX(3,2*lap:end)+h,'r-','LineWidth',sizeReal); hold on;
ptr2 = plot3(AX(1,2*lap:end),AX(2,2*lap:end),AX(3,2*lap:end),'b-','LineWidth',sizeReal);

% Ponto inicial do trecho
pi1 = plot3(PX(1,2*lap),PX(2,2*lap),PX(3,2*lap)+h,'k^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2)); hold on;
pi2 = plot3(AX(1,2*lap),AX(2,2*lap),AX(3,2*lap),'k^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));


% Final positions
pend1 = plot3(PX(1,end),PX(2,end),PX(3,end),'ksq','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
pend2 = plot3(AX(1,end),AX(2,end),AX(3,end),'ksq','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));

% title('Terceira volta','FontSize',sizeTitle,'interpreter','Latex');
xlabel('$x$ [m]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('$y$ [m]','FontSize',sizeLabel,'Interpreter','latex');
zlabel('$z$ [m]','FontSize',sizeLabel,'interpreter','Latex');
axis([-2 2 -2 2 0 1.5])
grid on
box on
% 
% % legenda PT
% leg12 = legend([pi1 pend1 ptd1 ptd2 ptr1 ptr2],{'Posi\c{c}\~{a}o inicial','Posi\c{c}\~{a}o final','Trajet\''{o}ria desejada Pioneer 3-DX','Trajet\''{o}ria desejada AR.Drone 2.0',...
%     'Trajet\''{o}ria Pioneer 3-DX','Trajet\''{o}ria AR.Drone 2.0'});
% leg12.FontSize = sizeLegenda;
% leg12.Location = location;
% set(leg12,'Interpreter','latex');

% legenda EN
leg12 = legend([pi1 pend1 ptd1 ptd2 ptr1 ptr2],{'Initial position','Final Position','Desired trajectory for the UGV','Desired trajectory for the UAV',...
    'Real trajectory for the UGV','Real trajectory for the UAV'});
leg12.FontSize = sizeLegenda;
leg12.Location = location;
set(leg12,'Interpreter','latex');


%% ###################################################################################
% 
%% Separate trajectory in 2 parts
% % Total de 2 voltas, a ideia é plotar cada volta separadamente
% totalLap  = 2;       % total de voltas 
% pedacim   = 4;       % total de "partes" para separar. Cada volta tem 4 partes
% lap = floor(pedacim/(4*totalLap)*length(AX));
% % :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
% % primeira volta 
% figure;
% % subplot(2,3,1)
% % percurso desejado
% plot3(PXd(1,1:lap),PXd(2,1:lap),PXd(3,1:lap)+h,'r--','LineWidth',sizeReal); hold on;
% plot3(AXd(1,1:lap),AXd(2,1:lap),AXd(3,1:lap),'b--','LineWidth',sizeReal);
% 
% % percurso realizado
% plot3(PX(1,1:lap),PX(2,1:lap),PX(3,1:lap)+h,'r-','LineWidth',sizeReal); hold on;
% plot3(AX(1,1:lap),AX(2,1:lap),AX(3,1:lap),'b-','LineWidth',sizeReal);
% 
% % Ponto final do trecho
% plot3(PX(1,lap),PX(2,lap),PX(3,lap)+h,'ksq','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2)); hold on;
% plot3(AX(1,lap),AX(2,lap),AX(3,lap),'ksq','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
% 
% % Initial positions
% plot3(PX(1,1),PX(2,1),PX(3,1)+h,'k^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
% plot3(AX(1,1),AX(2,1),AX(3,1),'k^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
% 
% % title('Primeira volta','FontSize',sizeTitle,'interpreter','Latex');
% xlabel('$x$ [m]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('$y$ [m]','FontSize',sizeLabel,'Interpreter','latex');
% zlabel('$z$ [m]','FontSize',sizeLabel,'interpreter','Latex');
% axis([-2 2 -2 2 0 2])
% grid on
% box on
% 
% % leg11 = legend([ pi1 pd1 pd2 pr1 pr2],{'Posi\c{c}\~{a}o inicial','Trajet\''{o}ria desejada Pioneer 3-DX', ...
% %     'Trajet\''{o}ria desejada AR.Drone 2.0','Trajet\''{o}ria Pioneer 3-DX','Trajet\''{o}ria AR.Drone 2.0'});
% % leg11.FontSize = sizeLegenda;
% % leg11.Location = location;
% % set(leg11,'Interpreter','latex');
% 
% %::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
% % segunda volta 
% figure
% % subplot(2,3,2)
% % percurso desejado
% pd1 = plot3(PXd(1,lap:2*lap),PXd(2,lap:2*lap),PXd(3,lap:2*lap)+h,'r--','LineWidth',sizeReal); hold on;
% pd2 = plot3(AXd(1,lap:2*lap),AXd(2,lap:2*lap),AXd(3,lap:2*lap),'b--','LineWidth',sizeReal);
% 
% % percurso realizado
% pr1 = plot3(PX(1,lap:2*lap),PX(2,lap:2*lap),PX(3,lap:2*lap)+h,'r-','LineWidth',sizeReal); hold on;
% pr2 = plot3(AX(1,lap:2*lap),AX(2,lap:2*lap),AX(3,lap:2*lap),'b-','LineWidth',sizeReal);
% 
% % Ponto inicial do trecho
% pi1 = plot3(PX(1,lap),PX(2,lap),PX(3,lap)+h,'k^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2)); hold on;
% pi2 = plot3(AX(1,lap),AX(2,lap),AX(3,lap),'k^','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
% 
% % Ponto final do trecho
% pf1 = plot3(PX(1,2*lap),PX(2,2*lap),PX(3,2*lap)+h,'ksq','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2)); hold on;
% pf2 = plot3(AX(1,2*lap),AX(2,2*lap),AX(3,2*lap),'ksq','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));
% 
% % title('Segunda volta','FontSize',sizeTitle,'interpreter','Latex');
% xlabel('$x$ [m]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('$y$ [m]','FontSize',sizeLabel,'Interpreter','latex');
% zlabel('$z$ [m]','FontSize',sizeLabel,'interpreter','Latex');
% axis([-2 2 -2 2 0 2])
% grid on
% box on
% 
% % legenda
% leg12 = legend([pi1 pf1 pd1 pd2 pr1 pr2],{'Posi\c{c}\~{a}o inicial',...
%     'Posi\c{c}\~{a}o final','Trajet\''{o}ria desejada Pioneer 3-DX',...
%     'Trajet\''{o}ria desejada AR.Drone 2.0',...
%     'Trajet\''{o}ria Pioneer 3-DX','Trajet\''{o}ria AR.Drone 2.0'});
% leg12.FontSize = sizeLegenda;
% leg12.Location = location;
% set(leg12,'Interpreter','latex');

