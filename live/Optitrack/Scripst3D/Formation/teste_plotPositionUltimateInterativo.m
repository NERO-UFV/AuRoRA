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
P = Pioneer3DX;
A = ArDrone;
LF = LineFormation3D;

%% Carrega dados
% data = load('FL2dtrajElipseExp_20181016T104710.txt');
data = load('FL3d_PositionExp20181011T194510.txt');   % position ultimate

%% Atribuição de variáveis
% data = data(1:1000,:);
%Tempo da simulação
time  = data(:,end);                % tempo (s)

% Robô 1
P.pPos.Xd   = data(:,(1:12));       % postura desejada
P.pPos.X    = data(:,12+(1:12));    % postura real
P.pSC.Ud    = data(:,24+(1:2));     % sinal de controle
P.pSC.U     = data(:,26+(1:2));     % velocidades do robô
P.pPos.Xtil = P.pPos.Xd - P.pPos.X; % erro de postura

% Robô 2
A.pPos.Xd   = data(:,28+(1:12));
AX    = data(:,40+(1:12));
A.pSC.Ud    = data(:,52+(1:4));
A.pSC.U     = data(:,56+(1:4));
A.pPos.Xtil = A.pPos.Xd - AX;

% Dados da Formação
Qd           = data(:,60+(1:6));   % desejado
Q            = data(:,66+(1:6));   % realizado

Qtil         = Qd - Q;

% Vetor de posição dos robôs: LF.pPos.X = [x1 y1 rho1 theta1 psi1 x2 y2 z2 rho2 theta2 psi2]
% Vetor da formação: LF.pPos.Q = [xf yf rhof alfaf]
for k=1:length(time)
    LF.pPos.X = [P.pPos.X(k,1:3) AX(k,1:3)]';
    %     LF.mDirTrans;
    %     Q(:,:)   = LF.pPos.Q';
    LF.pPos.Q = Q(k,:)';
    LF.pPos.Qd = Qd(k,:)';
    LF.mInvTrans;
    PXd(:,k) = LF.pPos.Xd;
end


%% PLOTA RESULTADOS
sizeLineDesired   = 2;%'default';    % largura da linha
sizeLineReal      = 0.8;
sizeLegend = 20;  % tamanho da fonte
sizeSymbol = 2;   % tamanho dos símbolos
sizeLabel  = 16;

%% Percurso dos robôs
figure;
axis equal
hold on, grid on;
box on
view(-45,10)
% pause(4)
% Posições desejadas de cada robô
% pd  = plot3(Qd(:,1),Qd(:,2),Qd(:,3),'ro','LineWidth',2);
% pdd2 = plot3(PXd(4,:),PXd(5,:),PXd(6,:),'bx','LineWidth',2);
pd  = plot3(PXd(1,1),PXd(2,1),PXd(3,1),'ro','LineWidth',2);
pdd2 = plot3(PXd(4,1),PXd(5,1),PXd(6,1),'bx','LineWidth',2);

% Pontos iniciais
plot3(P.pPos.X(1,1),P.pPos.X(1,2),P.pPos.X(1,3),'r^','LineWidth',sizeLineReal);
plot3(AX(1,1),AX(1,2),AX(1,3),'b^','LineWidth',sizeLineReal);


% primeiro ponto do percurso realizado
% pd2 = plot3(Q(1,1),Q(2,1),Q(3,1),'cx','LineWidth',sizeLineReal);
p1 = plot3(P.pPos.X(1,1),P.pPos.X(1,2),P.pPos.X(1,3),'rx','LineWidth',sizeLineReal);
p2 = plot3(AX(1,1),AX(1,2),AX(1,3),'bo','LineWidth',sizeLineReal);

axis([-2 2 -1.5 1.5 0 2]);

pdb  = p1;
pdb2 = p2;

timebase = 15;       % tempo para cada ponto
tend     = 105.455;  % tempo em que finaliza os pontos desejados [ajustado na marra (rs)]
trail    = 100;      % pontos de rastro
trate    = 0.2;%1/30;     % taxa de atualização da simulação
cont     = 1;        % contador
k        = 1;        % índice para varrer os vetores de dados

pause(4)

t = tic;   % tempo atual
% plota robos e linhas de formação
while k < length(time)
    tic
    % Plota próximos pontos desejados
    if toc(t)> trate
        t = tic;
        
        if time(k)<tend
            pd   = plot3(PXd(1,k),PXd(2,k),PXd(3,k),'ro','LineWidth',2); % pioneer
            pdd2 = plot3(PXd(4,k),PXd(5,k),PXd(6,k),'bx','LineWidth',2); % drone
        end
        % Escurece alvos já atingidos
        if time(k)> tend
            delete(pdd2); delete(pd);
            
%             delete(pdb); delete(pdb2);
            
            pdb  = plot3(PXd(1,k),PXd(2,k),PXd(3,k),'ko','LineWidth',2); % pioneer
            pdb2 = plot3(PXd(4,k),PXd(5,k),PXd(6,k),'k.','LineWidth',2); % drone
            
        elseif time(k)> cont*timebase
            delete(pdb); delete(pdb2);            
            pdb  = plot3(PXd(1,1:k),PXd(2,1:k),PXd(3,1:k),'ko','LineWidth',2); % pioneer
            pdb2 = plot3(PXd(4,1:k),PXd(5,1:k),PXd(6,1:k),'kx','LineWidth',2); % drone
            cont = cont + 1;      
        end
        
        % Obtenção da posição do centro dos robôs
        P.pPos.Xc([1 2 6]) = P.pPos.X(k,[1 2 6])' - ...
            [P.pPar.a*cos(P.pPos.X(k,6)); P.pPar.a*sin(P.pPos.X(k,6)); 0];
        % Posição atual do drone
        A.pPos.X = AX(k,:)';
        
        %     delete(pd2)
        %     %
        %     pd2 = plot(Q(1,1:k),Q(2,1:k),'g-','LineWidth',sizeLineReal);
        %     % percurso realizado
        delete(p1);
        delete(p2);
        
        %plota rastro
        p1 = plot3(P.pPos.X(1:k,1),P.pPos.X(1:k,2),P.pPos.X(1:k,3),'r-','LineWidth',sizeLineReal);
        p2 = plot3(AX(1:k,1),AX(1:k,2),AX(1:k,3),'b-','LineWidth',sizeLineReal);
        %
        %      % Plota cauda
        %     if k < trail     % porque pontos iniciais não possuem valores para rastro
        %         p1 = plot3(P.pPos.X(1:k,1),P.pPos.X(1:k,2),P.pPos.X(1:k,3),'r-','LineWidth',sizeLineReal);
        %         p2 = plot3(AX(1:k,1),AX(1:k,2),AX(1:k,3),'b-','LineWidth',sizeLineReal);
        %     else
        %         p1 = plot3(P.pPos.X(k-(trail-1):k,1),P.pPos.X(k-(trail-1):k,2),P.pPos.X(k-(trail-1):k,3),'r-','LineWidth',sizeLineReal);
        %         p2 = plot3(AX(k-(trail-1):k,1),AX(k-(trail-1):k,2),AX(k-(trail-1):k,3),'b-','LineWidth',sizeLineReal);
        %     end
        
        % Labels dos eixos
        xlabel('$x$ [m]','FontSize',sizeLabel,'interpreter','Latex')
        ylabel('$y$ [m]','FontSize',sizeLabel,'Interpreter','latex');
        zlabel('$z$ [m]','FontSize',sizeLabel,'Interpreter','latex');
        
        P.mCADdel;
        % plota  robos
        P.mCADplot(1,'k');           % pioner (escala, cor)
        A.mCADplot;                  % ardrone
        axis([-2 2 -1.5 1.5 0 2]);
        %     view(-40,20)
        view(-45,10)
        drawnow
        
        if time(k)>tend
        k = k+8;
        else
        k = k+15;
        end
        toc
    end
    
    
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
