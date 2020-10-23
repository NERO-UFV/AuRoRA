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
Pioneer = Pioneer3DX;
Pioneer.pPar.a = 0;
Drone = ArDrone;
LF = LineFormation3D;

%% Carrega dados
data = load('FL3d_TrajElipseUltimate20181011T183738.txt');
% data = load('FL3d_TrajElipseExp20181011T122543.txt');

%% Atribuição de variáveis
% data = data(1:1000,:);
%Tempo da simulação
time  = data(:,end);                % tempo (s)

% Robô 1
P.Xd   = data(:,(1:12));       % postura desejada
P.X    = data(:,12+(1:12));    % postura real
P.Ud   = data(:,24+(1:2));     % sinal de controle
P.U    = data(:,26+(1:2));     % velocidades do robô
P.Xtil = P.Xd - P.X; % erro de postura

% Robô 2
A.Xd   = data(:,28+(1:12));
A.X    = data(:,40+(1:12));
A.Ud   = data(:,52+(1:4));
A.U    = data(:,56+(1:4));
A.Xtil = A.Xd - A.X;

% Dados da Formação
Qd           = data(:,60+(1:6));   % desejado
Q            = data(:,66+(1:6));   % realizado

Qtil         = Qd - Q;


% Vetor de posição dos robôs: LF.pPos.X = [x1 y1 rho1 theta1 psi1 x2 y2 z2 rho2 theta2 psi2]
% Vetor da formação: LF.pPos.Q = [xf yf rhof alfaf]
% for k=1:length(time)
%      LF.pPos.X = [P.pPos.X(k,1:3) AX(k,1:3)]';
%     %     LF.mDirTrans;
%     %     Q(:,:)   = LF.pPos.Q';
%     LF.pPos.Q = Q(k,:)';
%     LF.pPos.Qd = Qd(k,:)';
%     LF.mInvTrans;
%     PXd(:,k) = LF.pPos.Xd;
% end


%% PLOTA RESULTADOS
sizeLineDesired   = 1.5;%'default';    % largura da linha
sizeLineReal      = 0.8;
sizeLegend        = 16;  % tamanho da fonte
sizeSymbol        = 2;   % tamanho dos símbolos
sizeLabel         = 16;
location          = 'NorthEast';

%% Trajetória dos robôs
figure;
axis equal
hold on, grid on;
box on
% view(-40,20)
% view(0,15)
% view(-20,17)
 view(0,90)
% pause(4)
% Posições desejadas da formação
pform  = plot3(Qd(1,1),Qd(1,2),Qd(1,3),'g*','MarkerSize',12,'LineWidth',2);
pformt  = plot3(Qd(:,1),Qd(:,2),Qd(:,3),'k--','LineWidth',sizeLineDesired);

% pdd2 = plot3(PXd(4,:),PXd(5,:),PXd(6,:),'bx','LineWidth',2);
% pd  = plot3(PXd(1,1),PXd(2,1),PXd(3,1),'ro','LineWidth',2);
% pdd2 = plot3(PXd(4,1),PXd(5,1),PXd(6,1),'bx','LineWidth',2);
% Pontos iniciais
pinit = plot3(P.X(1,1),P.X(1,2),P.X(1,3),'k^','LineWidth',sizeLineReal);
plot3(A.X(1,1),A.X(1,2),A.X(1,3),'k^','LineWidth',sizeLineReal);


% primeiro ponto do percurso realizado
% pd2 = plot3(Q(1,1),Q(2,1),Q(3,1),'cx','LineWidth',sizeLineReal);
p1 = plot3(P.X(1,1),P.X(1,2),P.X(1,3),'r-','LineWidth',sizeLineReal);
p2 = plot3(A.X(1,1),A.X(1,2),A.X(1,3),'b-','LineWidth',sizeLineReal);

leg1 = legend([pinit pformt],{'Posi\c{c}\~{a}o inicial',...
    'Trajet\''{o}ria desejada'});

leg1.FontSize = sizeLegend;
leg1.Location = location;
set(leg1,'Interpreter','latex');

axis([-2 2 -1.5 1.5 0 2]);

pdb  = p1;
pdb2 = p2;
% tend    =   20;    % tempo para mudar o passo da simulação
trail    = 100;      % pontos de rastro
trate    = 0.2;%1/30;     % taxa de atualização da simulação
k        = 1;        % índice para varrer os vetores de dados

pause(5)
delete(leg1);
t = tic;   % tempo atual
% plota robos e linhas de formação
while k < length(time)
    tic
    % Força um tempo mínimo para manter uniforme a simulação
    if toc(t)> trate
        t = tic;
        
        delete(pform);
        pform  = plot3(Qd(k,1),Qd(k,2),Qd(k,3),'g*','MarkerSize',12,'LineWidth',2);
        
        %         pd   = plot3(P.Xd(k,1),P.Xd(k,2),P.Xd(k,3),'ro','LineWidth',2); % pioneer
        %         pdd2 = plot3(A.Xd(k,1),A.Xd(k,2),A.Xd(k,3),'bx','LineWidth',2); % drone
        
        
        % Obtenção da posição do centro dos robôs
        Pioneer.pPos.Xc([1 2 6]) = P.X(k,[1 2 6])' - ...
            [Pioneer.pPar.a*cos(P.X(k,6)); Pioneer.pPar.a*sin(P.X(k,6)); 0];
        % Posição atual do drone
        Drone.pPos.X = A.X(k,:)';
        
        delete(p1);
        delete(p2);
        
        %plota rastro
        p1 = plot3(P.X(1:k,1),P.X(1:k,2),P.X(1:k,3),'r-','LineWidth',sizeLineReal);
        p2 = plot3(A.X(1:k,1),A.X(1:k,2),A.X(1:k,3),'b-','LineWidth',sizeLineReal);
        %
        % Plota cauda
        %         if k < trail     % porque pontos iniciais não possuem valores para rastro
        %             p1 = plot3(P.X(1:k,1),P.X(1:k,2),P.X(1:k,3),'r-','LineWidth',sizeLineReal);
        %             p2 = plot3(A.X(1:k,1),A.X(1:k,2),A.X(1:k,3),'b-','LineWidth',sizeLineReal);
        %         else
        %             p1 = plot3(P.X(k-(trail-1):k,1),P.X(k-(trail-1):k,2),P.X(k-(trail-1):k,3),'r-','LineWidth',sizeLineReal);
        %             p2 = plot3(A.X(k-(trail-1):k,1),A.X(k-(trail-1):k,2),A.X(k-(trail-1):k,3),'b-','LineWidth',sizeLineReal);
        %         end
        
        % Labels dos eixos
        xlabel('$x$ [m]','FontSize',sizeLabel,'interpreter','Latex')
        ylabel('$y$ [m]','FontSize',sizeLabel,'Interpreter','latex');
        zlabel('$z$ [m]','FontSize',sizeLabel,'Interpreter','latex');
      
        Pioneer.mCADdel;
        % plota  robos
        Pioneer.mCADplot(1,'k');           % pioner (escala, cor)
        Drone.mCADplot;                  % ardrone
        axis([-2 2 -1.5 1.5 0 2]);
%             view(-40,20)
%         view(0,15)
% view(-20,17)
        drawnow
        
        %         if time(k)>tend
%         k = k+1;
        %         else
                k = k+10;
        %         end
        toc
    end
    
    
end
close
