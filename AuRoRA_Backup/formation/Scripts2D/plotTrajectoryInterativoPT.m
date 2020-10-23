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
% data = load('FL2dtrajElipseExp_20181016T104710.txt');
% data = load('FL2dtrajElipsExp_20181016T104710.txt');

%% Atribuição de variáveis
% data = data(1:1000,:);
%Tempo da simulação
time  = data(:,end);                % tempo (s)

% Robô 1
P1.pPos.Xd   = data(:,(1:12));       % postura desejada
P1.pPos.X    = data(:,12+(1:12));    % postura real
P1.pSC.Ud    = data(:,24+(1:2));     % sinal de controle
P1.pSC.U     = data(:,26+(1:2));     % velocidades do robô
P1.pPos.Xtil = P1.pPos.Xd - P1.pPos.X; % erro de postura

% Robô 2
P2.pPos.Xd   = data(:,28+(1:12));
P2.pPos.X    = data(:,40+(1:12));
P2.pSC.Ud    = data(:,52+(1:2));
P2.pSC.U     = data(:,54+(1:2));
P2.pPos.Xtil = P2.pPos.Xd - P2.pPos.X;

% Dados da Formação
Qd           = data(:,56+(1:4));
LF.pPos.Qtil = data(:,60+(1:4));


% Vetor de posição dos robôs: LF.pPos.X = [x1 y1 rho1 theta1 psi1 x2 y2 z2 rho2 theta2 psi2]
% Vetor da formação: LF.pPos.Q = [xf yf rhof alfaf]
for k=1:length(time)
    LF.pPos.X = [P1.pPos.X(k,1:6) P2.pPos.X(k,1:6)];
    LF.mDirTrans;
    Q(:,k)   = LF.pPos.Q';
    
    LF.pPos.Qd = Qd(k,:)';
    LF.mInvTrans('d');
    PXd(:,k) = LF.pPos.Xd;
end


%% PLOTA RESULTADOS
sizeLineDesired   = 2;%'default';    % largura da linha
sizeLineReal      = 1.3;
sizeLegend = 20;  % tamanho da fonte
sizeSymbol = 2;   % tamanho dos símbolos
sizeLabel  = 16;

%% Posição dos robôs
figure;
axis([-3 3 -3 3]);
axis equal
hold on, grid on;
box on
pause(4)
% trajetória da formação desejada
pd  = plot(Qd(:,1),Qd(:,2),'k--','LineWidth',2);


% primeiro ponto da trajetoria real
pd2 = plot(Q(1,1),Q(2,1),'c-','LineWidth',sizeLineReal);
p1 = plot(P1.pPos.X(1,1),P1.pPos.X(1,2),'r-','LineWidth',sizeLineDesired);
p2 = plot(P2.pPos.X(1,1),P2.pPos.X(1,2),'b-','LineWidth',sizeLineReal);

% plota robos e linhas de formação
for k = 1:length(time)
    

    % Obtenção da posição do centro dos robôs
    P1.pPos.Xc([1 2 6]) = P1.pPos.X(k,[1 2 6])' - ...
        [P1.pPar.a*cos(P1.pPos.X(k,6)); P1.pPar.a*sin(P1.pPos.X(k,6)); 0];
    
    P2.pPos.Xc([1 2 6]) = P2.pPos.X(k,[1 2 6])' - ...
        [P2.pPar.a*cos(P2.pPos.X(k,6)); P2.pPar.a*sin(P2.pPos.X(k,6)); 0];
    
    delete(pd2)
    % trajetória da formação real
    pd2 = plot(Q(1,1:k),Q(2,1:k),'g-','LineWidth',sizeLineReal);
    % percurso realizado
    delete(p1);
    delete(p2);
    
    if k<51
    p1 = plot(P1.pPos.X(1:k,1),P1.pPos.X(1:k,2),'r-','LineWidth',sizeLineReal);
    p2 = plot(P2.pPos.X(1:k,1),P2.pPos.X(1:k,2),'b-','LineWidth',sizeLineReal);
    else
    p1 = plot(P1.pPos.X(k-50:k,1),P1.pPos.X(k-50:k,2),'r-','LineWidth',sizeLineReal);
    p2 = plot(P2.pPos.X(k-50:k,1),P2.pPos.X(k-50:k,2),'b-','LineWidth',sizeLineReal);
    end
    % title('Posição dos Robôs','fontSize',lt);
    xlabel('$x$ [m]','FontSize',sizeLabel,'interpreter','Latex'),ylabel('$y$ [m]','FontSize',sizeLabel,'Interpreter','latex');
    
    
    % plota trianglinho
    %         P1.mCADplot2D('r');
    %         P2.mCADplot2D('b');
    P1.mCADdel;
    P2.mCADdel;
    %     % plota pioneer
    P1.mCADplot(1,'r');
    P2.mCADplot(1,'b');
    axis([-3 3 -3 3]);
    view(-90,90)
    drawnow
end
close

%% Posição interativa!!
% Xpos = [];
% hFig = figure(1);
% set(hFig, 'Position', [50 100 1200 120]), shg
% pause(4)
% subplot(141)
% hp1 = plot(time,PXd(1,:),'r--','LineWidth',sizeLineDesired);hold on;
% plot(time(1),P1.pPos.X(1,1),'b-','LineWidth',sizeLineReal);
% xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
%
% ylabel('$x_1$ [m]','FontSize',sizeLabel,'interpreter','Latex');
% axis([0 (time(end)-5) -3 3])
% grid on
%
% subplot(142)
% hp2 = plot(time,PXd(2,:),'r--','LineWidth',sizeLineDesired);hold on;
% plot(time(1),P1.pPos.X(1,2),'b-','LineWidth',sizeLineReal);
% xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
%
% ylabel('$y_1$ [m]','FontSize',sizeLabel,'interpreter','Latex');
% axis([0 (time(end)-5) -3 3])
% grid on
%
% subplot(143)
% hp3 = plot(time,PXd(3,:),'r--','LineWidth',sizeLineDesired);hold on;
% plot(time(1),P2.pPos.X(1,1),'b-','LineWidth',sizeLineReal);
% xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
%
% ylabel('$x_2$ [m]','FontSize',sizeLabel,'interpreter','Latex');
% axis([0 (time(end)-5) -3 3])
% grid on
%
% subplot(144)
% hp4 = plot(time,PXd(4,:),'r--','LineWidth',sizeLineDesired);hold on;
% plot(time(1),P1.pPos.X(1,2),'b-','LineWidth',sizeLineReal);
% xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% ylabel('$y_2$ [m]','FontSize',sizeLabel,'interpreter','Latex');
% axis([0 (time(end)-5) -3 3])
% grid on
%
% k  = 1;
% tsimp = tic;
%
% while k<=length(time-5)
%
%     if toc(tsimp)>0.2
%         tsimp = tic;
%
%         % position vector
%         Xpos = [Xpos [P1.pPos.X(k,1:2)';P2.pPos.X(k,1:2)';time(k)]];
%
%
%         delete(hp1)
%         delete(hp2)
%         delete(hp3)
%         delete(hp4)
%
%         subplot(141)
%         hp1 =  plot(time,PXd(1,:),'r--','LineWidth',sizeLineDesired);hold on;
%         plot(Xpos(end,:),Xpos(1,:),'b-','LineWidth',sizeLineReal);
% %         xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% %         ylabel('$x_1$ [m]','FontSize',sizeLabel,'interpreter','Latex');
%         axis([0 (time(end)-5) -3 3])
%         grid on
%
%         subplot(142)
%         hp2 = plot(time,PXd(2,:),'r--','LineWidth',sizeLineDesired);hold on;
%         plot(Xpos(end,:),Xpos(2,:),'b-','LineWidth',sizeLineReal);
% %         xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% %         ylabel('$y_1$ [m]','FontSize',sizeLabel,'interpreter','Latex');
%         axis([0 (time(end)-5) -3 3])
%         grid on
%
%         subplot(143)
%         hp3 = plot(time,PXd(3,:),'r--','LineWidth',sizeLineDesired);hold on;
%         plot(Xpos(end,:),Xpos(3,:),'b-','LineWidth',sizeLineReal);
% %         xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% %         ylabel('$x_2$ [m]','FontSize',sizeLabel,'interpreter','Latex');
%         axis([0 (time(end)-5) -3 3])
%         grid on
%
%         subplot(144)
%         hp4 = plot(time,PXd(4,:),'r--','LineWidth',sizeLineDesired);hold on;
%         plot(Xpos(end,:),Xpos(4,:),'b-','LineWidth',sizeLineReal);
% %         xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% %         ylabel('$y_2$ [m]','FontSize',sizeLabel,'interpreter','Latex');
%         axis([0 (time(end)-5) -3 3])
%
%         grid on
%
%         drawnow
%
%         k = k+1;
%
%     end
% end
% close
