%% Rotina para plotar gráficos de Formacaolinha2d trajetória
clear all
close all
clc
%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Declara os robos
P1 = Pioneer3DX;
P2 = Pioneer3DX;
LF = LineFormation2D('center');

%% Carrega dados
data = load('FL2dCompensador_20180503T202326.txt');   % experimento
% data = load('FL2dCompensador_20180504T092330.txt'); % simulacao

%% Atribuição de variáveis
% data = data(1:1000,:);
%Tempo da simulação
time  = data(:,end);                % tempo (s)

% Robô A
P1.pPos.Xd   = data(:,(1:12));       % postura desejada
P1.pPos.X    = data(:,12+(1:12));    % postura real
P1.pSC.Ud    = data(:,24+(1:2));     % sinal de controle
P1.pSC.U     = data(:,26+(1:2));     % velocidades do robô
P1.pPos.Xtil = P1.pPos.Xd - P1.pPos.X; % erro de postura

% Robô B
P2.pPos.Xd   = data(:,28+(1:12));
P2.pPos.X    = data(:,40+(1:12));
P2.pSC.Ud    = data(:,52+(1:2));
P2.pSC.U     = data(:,54+(1:2));
P2.pPos.Xtil = P2.pPos.Xd - P2.pPos.X;

% % Dados da Formação
% LF.pPos.Qd    = data(:,56+(1:4));
% LF.pPos.Qtil = data(:,60+(1:4));

xsq = 1.2;
ysq = 1.24;

Qd = [3*xsq 3*ysq xsq 0]';    % formação desejada
% Vetor de posição dos robôs: LF.pPos.X = [x1 y1 rho1 theta1 psi1 x2 y2 z2 rho2 theta2 psi2]
% Vetor da formação: LF.pPos.Q = [xf yf rhof alfaf]
for k=1:length(time)
    LF.pPos.X = [P1.pPos.X(k,1:6) P2.pPos.X(k,1:6)];
    LF.mDirTrans;
    Q(:,k) = LF.pPos.Q';
    Qtil(:,k) = Qd - Q(:,k);
end

%% PLOTA RESULTADOS
sizeLineDesired   = 2;%'default';    % largura da linha
sizeLineReal      = 1.3;
sizeLegend = 16;  % tamanho da fonte
sizeSymbol = [15 2];   % tamanho dos símbolos
sizeLabel  = 18;


%% Posição dos robôs
figure(1);
hold on, grid on;
box on
axis equal
axis([-0.5 6 -2 4]);

% percurso realizado
p1 = plot(P1.pPos.X(:,1),P1.pPos.X(:,2),'r-','LineWidth',sizeLineReal);
p2 = plot(P2.pPos.X(:,1),P2.pPos.X(:,2),'b-','LineWidth',sizeLineReal);
% title('Posição dos Robôs','fontSize',lt);
xlabel('$x$ [m]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('$y$ [m]','FontSize',sizeLabel,'Interpreter','latex');

%  formação desejada
LF.pPos.Qd = Qd;
LF.mInvTrans('d');
xd = [LF.pPos.Xd(1) LF.pPos.Xd(3)];
yd = [LF.pPos.Xd(2) LF.pPos.Xd(4)];
pd = plot(xd,yd,'k-x','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));hold on;
% pd  = plot(Qd(1),Qd(2),'kx','MarkerSize',sizeSymbol(1),'LineWidth',sizeSymbol(2));

% plota robos e linhas de formação
for k = 1:150:length(time)
    
    % Obtenção da posição do centro dos robôs
    P1.pPos.Xc([1 2 6]) = P1.pPos.X(k,[1 2 6])' - ...
        [P1.pPar.a*cos(P1.pPos.X(k,6)); P1.pPar.a*sin(P1.pPos.X(k,6)); 0];
    
    P2.pPos.Xc([1 2 6]) = P2.pPos.X(k,[1 2 6])' - ...
        [P2.pPar.a*cos(P2.pPos.X(k,6)); P2.pPar.a*sin(P2.pPos.X(k,6)); 0];
    
    
    % plota trianglinho
    P1.mCADplot2D('r');
    P2.mCADplot2D('b');
    
    %     % plota pioneer
    %     A.mCADplot(1,'r');
    %     B.mCADplot(1,'b');
    
    % plot linhas da formação executadas
    x = [P1.pPos.X(k,1) P2.pPos.X(k,1)];
    y = [P1.pPos.X(k,2) P2.pPos.X(k,2)];
    
    pl = line(x,y);
    pl.Color = 'g';
    pl.LineStyle = '-';
    pl.LineWidth = 1.3;
    
    %     hold on
    
    %     % plot linhas da formação desejadas
    %     xd = [A.pPos.Xd(k,1) B.pPos.Xd(k,1)];
    %     yd = [A.pPos.Xd(k,2) B.pPos.Xd(k,2)];
    %
    %     pld = line(xd,yd);
    %     pld.Color = 'm';
    %     pld.LineStyle = '--';
    %     pld.LineWidth = 1;
end

% lg1 = legend([p1 p2 pl pld],{'Robo 1', 'Robo 2','Real formation','Formacao desejada'});
% lg1 = legend([p1 p2 pld pl pd pd2],{'Robot 1', 'Robot 2','Desired Formation','Real Formation','Desired trajectory','Real trajectory'});
lg1 = legend([p1 p2 pd],{'Rob\^{o} 1', 'Rob\^{o} 2','Forma\c c\~{a}o Desejada'});

lg1.FontSize = sizeLegend;
lg1.Location = 'SouthEast';
set(lg1,'Interpreter','latex');

%% Velocidades lineares
figure;
subplot(211)
plot(time(1:end-5),P1.pSC.Ud(1:end-5,1),'r--','LineWidth',sizeLineDesired),hold on;
plot(time(1:end-5),P1.pSC.U(1:end-5,1),'b','LineWidth',sizeLineReal);
legend({'$u_{1des}$','$u_{1real}$'},'FontSize',sizeLegend,'Interpreter','latex');
xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Velocidade [m/s]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);
ylim([0 0.6]);
grid on;

subplot(212)
plot(time(1:end-5),P2.pSC.Ud(1:end-5,1),'r--','LineWidth',sizeLineDesired),hold on;
plot(time(1:end-5),P2.pSC.U(1:end-5,1),'b','LineWidth',sizeLineReal);
% title('Velocidade Linear','fontSize',st);
xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Velocidade [m/s]','FontSize',sizeLabel,'interpreter','Latex');
% legend({'Robot 1', 'Robot 2'},'Interpreter','latex');
legend({'$u_{2des}$','$u_{2real}$'},'FontSize',sizeLegend,'Interpreter','latex');
xlim([0 time(end)]);
ylim([0 0.6]);
grid on

% comparação entre velocidades lineares dos robos
figure;
pu1 = plot(time(1:end-5),P1.pSC.U(1:end-5,1),'r--','LineWidth',sizeLineDesired);hold on;
pu2 = plot(time(1:end-5),P2.pSC.U(1:end-5,1),'b','LineWidth',sizeLineReal);
xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Velocidade [m/s]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);
% ylim([0 0.6]);
grid on;
lv0 = legend([pu1 pu2],{'Rob\^{o} 1', 'Rob\^{o} 2'});
lv0.FontSize = sizeLegend;
% lv.Location = 'SouthEast';
set(lv0,'Interpreter','latex');




% Velocidades Angulares -------------------------------------
figure;
subplot(211);
plot(time(1:end-5),P1.pSC.Ud(1:end-5,2),'r--','LineWidth',sizeLineDesired),hold on;
plot(time(1:end-5),P1.pSC.U(1:end-5,2),'b','LineWidth',sizeLineReal);
legend({'$\omega_{1des}$','$\omega_{1real}$'},'FontSize',12,'Interpreter','latex');
xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Velocidade [rad/s]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);
ylim([-1 1.3]);
grid on;

subplot(212);
plot(time(1:end-5),P2.pSC.Ud(1:end-5,2),'r--','LineWidth',sizeLineDesired),hold on;
plot(time(1:end-5),P2.pSC.U(1:end-5,2),'b','LineWidth',sizeLineReal);
legend({'$\omega _{2des}$','$\omega_{2real}$'},'FontSize',12,'Interpreter','latex');
grid on;
% title('Velocidade Angular','fontSize',st);
xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Velocidade [rad/s]','FontSize',sizeLabel,'interpreter','Latex');
% legend({'Robot 1', 'Robot 2'},'Interpreter','latex');
xlim([0 time(end)]);
ylim([-1 1.3]);

% comparação entre velocidades angulares dos robos
figure;
pw1 = plot(time(1:end-5),P1.pSC.U(1:end-5,2),'r--','LineWidth',sizeLineDesired);hold on;
pw2 = plot(time(1:end-5),P2.pSC.U(1:end-5,2),'b','LineWidth',sizeLineReal);
xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Velocidade [rad/s]','FontSize',sizeLabel,'interpreter','Latex');
lv = legend([pw1 pw2],{'Rob\^{o} 1', 'Rob\^{o} 2'});
lv.FontSize = sizeLegend;
% lv.Location = 'SouthEast';
set(lv,'Interpreter','latex');

xlim([0 time(end)]);
% ylim([0 0.6]);
grid on

%% Erros
% Erro da formação - posição
figure;
subplot(211)
hold on, grid on; box on;
plot(time(1:end-5),Qtil(1,1:end-5),'--','LineWidth',sizeLineReal);
plot(time(1:end-5),Qtil(2,1:end-5),'-.','LineWidth',sizeLineReal);
plot(time(1:end-5),Qtil(3,1:end-5),'-','LineWidth',sizeLineReal);

% title('Erro de posição','fontSize',st);
xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Erro [m]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);

lg3 = legend('$x_f$','$y_f$', '$\rho_f$');
% lg3 = legend('$x_f [m]$','$y_f [m]$', '$\rho_f [m]$');

lg3.FontSize = sizeLegend;
lg3.Location = 'NorthEast';
set(lg3,'Interpreter','latex');

% Angulo
% figure;
subplot(212)
hold on, grid on;
plot(time(1:end-5),Qtil(4,1:end-5),'LineWidth',sizeLineReal);
% title('Erro de posição','fontSize',st);
xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Erro [rad]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);
box on
lg4 = legend('$\alpha_f$');

lg4.FontSize = sizeLegend;
lg4.Location = 'NorthEast';
set(lg4,'Interpreter','latex');

%% Erro interativo!!

XX = [];
% figure(1);
hFig = figure(1);
set(hFig, 'Position', [20 200 1200 400])
pause(4)
% subplot(211)
yyaxis left
h1 = plot(time(end,1),Qtil(1,1),'LineWidth',sizeLineReal);
h2 = plot(time(end,1),Qtil(2,1),'LineWidth',sizeLineReal);
h3 = plot(time(end,1),Qtil(3,1),'LineWidth',sizeLineReal);
yyaxis right
h4 = plot(time(end,1),Qtil(4,1),'LineWidth',sizeLineReal);


tsim = tic;

for k=1:length(time-5)
%     if toc(tsim)>0.2
    tic
    tsim = tic;
    XX = [XX [Qtil(:,k);time(k)]];
    
    %     try
    delete(h1)
    delete(h2)
    delete(h3)
    delete(h4)
    %     end
    yyaxis left
%     subplot(211)
    h1 = plot(XX(end,:),XX(1,:),'b-','LineWidth',sizeLineReal); hold on
    h2 = plot(XX(end,:),XX(2,:),'r--','LineWidth',sizeLineReal);
    h3 = plot(XX(end,:),XX(3,:),'g-.','LineWidth',sizeLineReal);
    xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
    ylabel('Erro [m]','FontSize',sizeLabel,'interpreter','Latex');
    axis([0 (time(end)-5) -1 5])
    grid on
%     lg3 = legend('$x_f$','$y_f$', '$\rho_f$');
%     lg3.FontSize = sizeLegend;
%     lg3.Location = 'NorthEast';
%     set(lg3,'Interpreter','latex');
    
    yyaxis right
%     subplotr(212)
    h4 = plot(XX(end,:),XX(4,:),'-','LineWidth',sizeLineReal);
    axis([0 (time(end)-5) -1 5])
    grid on
    xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
    ylabel('Erro [rad]','FontSize',sizeLabel,'interpreter','Latex');
    box on
     lg4 = legend('$x_f$','$y_f$', '$\rho_f$','$\alpha_f$');
%     lg4 = legend('$\alpha_f$');
    lg4.FontSize = sizeLegend;
    lg4.Location = 'NorthEast';
    set(lg4,'Interpreter','latex');
    
    drawnow
%     pause(.06)
    toc
%         end
end
close all
