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
% data = load('FL2dCompensador_20180503T202326.txt');   % experimento
data = load('FL2dCompensador_20180504T092330.txt'); % simulacao

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

LF.pPos.Qd = [3*xsq 3*ysq xsq 0]';    % formação desejada
LF.mInvTrans('d');
PXd = ones(size(LF.pPos.Qd,1),length(time)).*LF.pPos.Xd;
% Vetor de posição dos robôs: LF.pPos.X = [x1 y1 rho1 theta1 psi1 x2 y2 z2 rho2 theta2 psi2]
% Vetor da formação: LF.pPos.Q = [xf yf rhof alfaf]
for k=1:length(time)
    LF.pPos.X = [P1.pPos.X(k,1:6) P2.pPos.X(k,1:6)];
    LF.mDirTrans;
    Q(:,k) = LF.pPos.Q';
    Qtil(:,k) = LF.pPos.Qd - Q(:,k);
end


%% PLOTA RESULTADOS
sizeLineDesired   = 2;%'default';    % largura da linha
sizeLineReal      = 1.3;
sizeLegend = 16;  % tamanho da fonte
sizeSymbol = [15 2];   % tamanho dos símbolos
sizeLabel  = 18;



% % %% Erro interativo!!
% % 
% % Xerro = [];
% % % figure(1);
% % hFig = figure(1);
% % set(hFig, 'Position', [20 200 1200 200]),shg
% % pause(4)
% % % subplot(211)
% % yyaxis left
% % he1 = plot(time(end,1),Qtil(1,1),'LineWidth',sizeLineReal);
% % he2 = plot(time(end,1),Qtil(2,1),'LineWidth',sizeLineReal);
% % he3 = plot(time(end,1),Qtil(3,1),'LineWidth',sizeLineReal);
% % yyaxis right
% % he4 = plot(time(end,1),Qtil(4,1),'LineWidth',sizeLineReal);
% % 
% % ke = 1;
% % tsime = tic;
% % 
% % while ke<=length(time-5)
% %     
% %     if toc(tsime)>0.2
% %         tsime = tic;
% %         Xerro = [Xerro [Qtil(:,ke);time(ke)]];
% %         
% %         delete(he1)
% %         delete(he2)
% %         delete(he3)
% %         delete(he4)
% %         
% %         yyaxis left
% %         
% %         he1 = plot(Xerro(end,:),Xerro(1,:),'b-','LineWidth',sizeLineReal); hold on
% %         he2 = plot(Xerro(end,:),Xerro(2,:),'r--','LineWidth',sizeLineReal);
% %         he3 = plot(Xerro(end,:),Xerro(3,:),'g-.','LineWidth',sizeLineReal);
% %         xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% %         ylabel('Erro [m]','FontSize',sizeLabel,'interpreter','Latex');
% %         axis([0 (time(end)-5) -1 5])
% %         grid on
% %         %     lg3 = legend('$x_f$','$y_f$', '$\rho_f$');
% %         %     lg3.FontSize = sizeLegend;
% %         %     lg3.Location = 'NorthEast';
% %         %     set(lg3,'Interpreter','latex');
% %         
% %         yyaxis right
% %         %     subplotr(212)
% %         he4 = plot(Xerro(end,:),Xerro(4,:),'-','LineWidth',sizeLineReal);
% %         axis([0 (time(end)-5) -1 5])
% %         grid on
% %         xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
% %         ylabel('Erro [rad]','FontSize',sizeLabel,'interpreter','Latex');
% %         box on
% %         lg4 = legend('$x_f$','$y_f$', '$\rho_f$','$\alpha_f$');
% %         %     lg4 = legend('$\alpha_f$');
% %         lg4.FontSize = sizeLegend;
% %         lg4.Location = 'NorthEast';
% %         set(lg4,'Interpreter','latex');
% %         
% %         drawnow
% %           ke = ke+1;   
% %     end
% % end
% %     close
    
    %% Posição interativa!!
    Xpos = [];
    hFig = figure(1);
    set(hFig, 'Position', [500 100 400 500]), shg
    pause(4)
    subplot(411)
    hp1 = plot(time,PXd(1,:),'r--','LineWidth',sizeLineDesired);hold on;
    plot(time(end,1),P1.pPos.X(1,1),'b-','LineWidth',sizeLineReal);
    ylabel('$x_1$ [m]','FontSize',sizeLabel,'interpreter','Latex');
    axis([0 (time(end)-5) -2 5])
    grid on
    
    subplot(412)
    hp2 = plot(time,PXd(2,:),'r--','LineWidth',sizeLineDesired);hold on;
    plot(time(end,1),P1.pPos.X(1,2),'b-','LineWidth',sizeLineReal);
    ylabel('$y_1$ [m]','FontSize',sizeLabel,'interpreter','Latex');
    axis([0 (time(end)-5) -2 5])
    grid on
    
    subplot(413)
    hp3 = plot(time,PXd(3,:),'r--','LineWidth',sizeLineDesired);hold on;
    plot(time(end,1),P2.pPos.X(1,1),'b-','LineWidth',sizeLineReal);
    ylabel('$x_2$ [m]','FontSize',sizeLabel,'interpreter','Latex');
    axis([0 (time(end)-5) -2 5])
    grid on
    
    subplot(414)
    hp4 = plot(time,PXd(1,:),'r--','LineWidth',sizeLineDesired);hold on;
    plot(time(end,1),P1.pPos.X(1,2),'b-','LineWidth',sizeLineReal);
    xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
    ylabel('$y_2$ [m]','FontSize',sizeLabel,'interpreter','Latex');
    axis([0 (time(end)-5) -2 5])
    grid on
    
    k  = 1;
    tsimp = tic;
    
    while k<=length(time-5)
        
        if toc(tsimp)>0.2
            tsimp = tic;
            
            % position vector
            Xpos = [Xpos [P1.pPos.X(k,1:2)';P2.pPos.X(k,1:2)';time(k)]];
            
            
            delete(hp1)
            delete(hp2)
            delete(hp3)
            delete(hp4)
            
            subplot(411)
            hp1 =  plot(time,PXd(1,:),'r--','LineWidth',sizeLineDesired);hold on;
            plot(Xpos(end,:),Xpos(1,:),'b-','LineWidth',sizeLineReal);
            ylabel('$x_1$ [m]','FontSize',sizeLabel,'interpreter','Latex');
            axis([0 (time(end)-5) -2 5])
            grid on
            
            subplot(412)
            hp2 = plot(time,PXd(2,:),'r--','LineWidth',sizeLineDesired);hold on;
            plot(Xpos(end,:),Xpos(2,:),'b-','LineWidth',sizeLineReal);
            ylabel('$y_1$ [m]','FontSize',sizeLabel,'interpreter','Latex');
            axis([0 (time(end)-5) -2 5])
            grid on
            
            subplot(413)
            hp3 = plot(time,PXd(3,:),'r--','LineWidth',sizeLineDesired);hold on;
            plot(Xpos(end,:),Xpos(3,:),'b-','LineWidth',sizeLineReal);
            ylabel('$x_2$ [m]','FontSize',sizeLabel,'interpreter','Latex');
            axis([0 (time(end)-5) -2 5])
            grid on
            
            subplot(414)
            hp4 = plot(time,PXd(4,:),'r--','LineWidth',sizeLineDesired);hold on;
            plot(Xpos(end,:),Xpos(4,:),'b-','LineWidth',sizeLineReal);
            xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
            ylabel('$y_2$ [m]','FontSize',sizeLabel,'interpreter','Latex');
            axis([0 (time(end)-5) -2 5])
            grid on
            
            drawnow
            
            k = k+1;
            
        end
    end
    close