%% Rotina para plotar gráficos de Formacaolinha2d com Desvio

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
LF = LineFormation2D;

%% Carrega dados
data = load('FL2dDesvio_20180504T114837.txt');

%% Atribuição de variáveis
%Tempo da simulação
time  = data(:,end);                % tempo (s)

% Robô A
P1.pPos.Xd   = data(:,(1:12))';       % postura desejada
P1.pPos.X    = data(:,12+(1:12))';    % postura real
P1.pSC.Ud    = data(:,24+(1:2))';     % sinal de controle
P1.pSC.U     = data(:,26+(1:2))';     % velocidades do robô
P1.pPos.Xtil = P1.pPos.Xd - P1.pPos.X; % erro de postura

% Robô B
P2.pPos.Xd   = data(:,28+(1:12))';
P2.pPos.X    = data(:,40+(1:12))';
P2.pSC.Ud    = data(:,52+(1:2))';
P2.pSC.U     = data(:,54+(1:2))';
P2.pPos.Xtil = P2.pPos.Xd - P2.pPos.X;

% Dados da Formação
% Qd = [3*1.24, 3*1.2, 1.24, 0]';   % Coordenadas desejadas dad formação
% 
% xsq = .89;
% ysq = .89;
% Qd = [9*xsq 0*ysq xsq pi/2]';   % Desvio

%Coordenadas da formação desejada:
% Inicial:
Qd0 = [8.01  0  .89  -pi/2]';
% Após 25% do percurso:
Qd1 = [8.01  0  .89   -pi]';
% Após 65% do percurso:
Qd2 = [8.01  0  .89  pi/2]';



% Vetor de posição dos robôs: LF.pPos.X = [x1 y1 z1 rho1 theta1 psi1 x2 y2 z2 rho2 theta2 psi2]
% Vetor da formação: LF.pPos.Q = [xf yf rhof alfaf]
for k=1:length(time)
    LF.pPos.X = [P1.pPos.X(1:6,k) P2.pPos.X(1:6,k)];
    LF.mDirTrans;
    Q(:,k) = LF.pPos.Q;
    
        if LF.pPos.Q(1)>=Qd0(1)*0.65  % obstáculo a X% do caminho                
            Qd(:,k) = Qd2;      
        elseif LF.pPos.Q(1)>=Qd0(1)*0.25 %        
            Qd(:,k) = Qd1;  
        else   
            Qd(:,k) = Qd0;
        end
    
    Qtil(:,k) = Qd(:,k) - Q(:,k);
end


% Correção de quadrante
for k=1:length(Q)
    if abs(Q(4,k)) > pi
        if Q(4,k) > 0
            Q(4,k) = Q(4,k) - 2*pi;
        else
            Q(4,k) = Q(4,k) + 2*pi;
        end
    end
     if abs(Qtil(4,k)) > pi
        if Qtil(4,k) > 0
            Qtil(4,k) = Qtil(4,k) - 2*pi;
        else
            Qtil(4,k) = Qtil(4,k) + 2*pi;
        end
     end
   
end

%% PLOTA RESULTADOS
sizeLineDesired  = 2;%'default';    % largura da linha
sizeLineReal     = 1;%'default';    % largura da linha
sizeLineDesired2 = 1;%'default';    % largura da linha
sizeLineReal2    = .5;%'default';    % largura da linha
sizeLine         = 1;%'default';    % largura da linha
sizeLabel        = 16;
sizeLegend       = 16;  % tamanho da fonte
ss               = 2;   % tamanho dos símbolos
step             = 80;
location         = 'NorthEast';
%% Posição dos robôs
figure(1);
axis equal
hold on, grid on;
box on
% percurso realizado
p1 = plot(P1.pPos.X(1,:),P1.pPos.X(2,:),'r-','LineWidth',sizeLine);
p2 = plot(P2.pPos.X(1,:),P2.pPos.X(2,:),'b-','LineWidth',sizeLine);
% title('Posição dos Robôs','fontSize',lt);
xlabel('$x$ [m]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('$y$ [m]','FontSize',sizeLabel,'Interpreter','latex');

%  Percurso da formação desejado
% pfd = plot(Qd(1,:),Qd(2,:),'r--','LineWidth',sizeLineDesired);
% pf = plot(Q(1,:),Q(2,:),'b-','LineWidth',sizeLine);
% Percurso dos robôs
% p1 = plot(P1.pPos.X(1,:),P1.pPos.X(2,:),'r-','LineWidth',sizeLine);hold on;
% p2 = plot(P2.pPos.X(1,:),P2.pPos.X(2,:),'b-','LineWidth',sizeLine);
% 

% % posições iniciais
% plot(A.pPos.X(1,1),A.pPos.X(1,2) ,'ro','LineWidth',ss); % início
% plot(B.pPos.X(1,1),B.pPos.X(1,2) ,'bo','LineWidth',ss); % inicio
% plot(A.pPos.X(end,1),A.pPos.X(end,2) ,'rx','LineWidth',ss); % chegada
% plot(B.pPos.X(end,1),B.pPos.X(end,2),'bx','LineWidth',ss);  % chegada

% plota robos e linhas de formação
for k = 1:step:length(time)
    
%     P1.pPos.Xc = P1.pPos.X(k,:);
%     P2.pPos.Xc = P2.pPos.X(k,:);
     % Obtenção da posição do centro dos robôs
    P1.pPos.Xc([1 2 6]) = P1.pPos.X([1 2 6],k) - ...
        [P1.pPar.a*cos(P1.pPos.X(6,k)); P1.pPar.a*sin(P1.pPos.X(6,k)); 0];
    
    P2.pPos.Xc([1 2 6]) = P2.pPos.X([1 2 6],k) - ...
        [P2.pPar.a*cos(P2.pPos.X(6,k)); P2.pPar.a*sin(P2.pPos.X(6,k)); 0];
    
    % plota pioneer 
    P1.mCADplot(1,'r');
    P2.mCADplot(1,'b');    

    % plot linhas da formação executadas
    x = [P1.pPos.X(1,k) P2.pPos.X(1,k)];
    y = [P1.pPos.X(2,k) P2.pPos.X(2,k)];
    
    pl = line(x,y);
    pl.Color = 'g';
    pl.LineStyle = '-';
    
%     hold on
%     % plot linhas da formação desejadas
%     xd = [A.pPos.Xd(k,1) B.pPos.Xd(k,1)];
%     yd = [A.pPos.Xd(k,2) B.pPos.Xd(k,2)];
%     
%     pld = line(xd,yd);
%     pld.Color = 'm';
%     pld.LineStyle = '--';
    
end

lg1 = legend([p1 p2 pl],{'Rob\^o 1', 'Rob\^o 2','Linha da Forma\c{c}\~ao'});

lg1.FontSize = sizeLegend;
lg1.Location = 'SouthEast';
set(lg1,'Interpreter','latex');


%% Posição da formação
figure;
% xf
subplot(221)
plot(time(1:end),Qd(1,:),'r--','LineWidth',sizeLineDesired),hold on;
plot(time(1:end),Q(1,:),'b-','LineWidth',sizeLineReal);

xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Posi\c{c}\~ao [m]','FontSize',sizeLabel,'interpreter','Latex');
legend({'$x_{fd}$','$x_{fr}$'},'FontSize',sizeLegend,'Interpreter','latex');
xlim([0 time(end)]);
% ylim([0 0.4]);
grid on
box on

% yf
subplot(222)
plot(time,Qd(2,:),'r--','LineWidth',sizeLineDesired),hold on;
plot(time,Q(2,:),'b-','LineWidth',sizeLineReal);

xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Posi\c{c}\~ao [m]','FontSize',sizeLabel,'interpreter','Latex');
legend({'$y_{fd}$','$y_{fr}$'},'FontSize',sizeLegend,'Interpreter','latex');
xlim([0 time(end)]);
% ylim([0 0.4]);
grid on
box on

% rhof
subplot(223)
plot(time,Qd(3,:),'r--','LineWidth',sizeLineDesired),hold on;
plot(time,Q(3,:),'b-','LineWidth',sizeLineReal);

xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Dist\^ancia [m]','FontSize',sizeLabel,'interpreter','Latex');
legend({'$\rho_{fd}$','$\rho_{fr}$'},'FontSize',sizeLegend,'Interpreter','latex');
xlim([0 time(end)]);
% ylim([0 0.4]);
grid on
box on

% alphaf
subplot(224)
plot(time,rad2deg(Qd(4,:)),'r--','LineWidth',sizeLineDesired),hold on;
plot(time,rad2deg(Q(4,:)),'b-','LineWidth',sizeLineReal);

xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('\^Angulo [graus]','FontSize',sizeLabel,'interpreter','Latex');
legend({'$\alpha_{fd}$','$\alpha_{fr}$'},'FontSize',sizeLegend,'Interpreter','latex');
xlim([0 time(end)]);
% ylim([-180 180]);
grid on
box on


%% Erro da formação
figure;
hold on, grid on;
subplot(211)
plot(time,Qtil(1,:),'--','LineWidth',sizeLineReal), hold on;
plot(time,Qtil(2,:),'-.','LineWidth',sizeLineReal);
plot(time,Qtil(3,:),'-','LineWidth',sizeLineReal);

xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Erro [m]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);
box on; grid on;

lg3 = legend('$x_f$','$y_f$', '$\rho_f$');
lg3.FontSize = sizeLegend;
lg3.Location = location;
set(lg3,'Interpreter','latex');


subplot(212)
plot(time,rad2deg(Qtil(4,:)),'LineWidth',sizeLine);
xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Erro [graus]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);
box on; grid on;

lg4 = legend('$\alpha_f$');
lg4.FontSize = sizeLegend;
lg4.Location = location;
set(lg4,'Interpreter','latex');

%% Velocidades agrupadas
% Velocidades lineares
figure;
subplot(211)
plot(time(1:end),P1.pSC.Ud(1,1:end),'r--','LineWidth',sizeLineDesired),hold on;
plot(time(1:end),P1.pSC.U(1,1:end),'b-','LineWidth',sizeLineReal);

plot(time(1:end),P2.pSC.Ud(1,1:end),'g--','LineWidth',sizeLineDesired2);
plot(time(1:end),P2.pSC.U(1,1:end),'k-','LineWidth',sizeLineReal2);
% title('Velocidade Linear','fontSize',st);
xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Velocidade linear [m/s]','FontSize',sizeLabel,'interpreter','Latex');
% legend({'Robot 1', 'Robot 2'},'Interpreter','latex');
legend({'$u_{1d}$','$u_{1r}$','$u_{2d}$','$u_{2r}$'},'FontSize',sizeLegend,'Interpreter','latex');
xlim([0 time(end)]);
% ylim([0 0.4]);
grid on
box on
% Velocidades Angulares -
% figure;
subplot(212);
plot(time(1:end),P1.pSC.Ud(2,1:end),'r--','LineWidth',sizeLineDesired),hold on;
plot(time(1:end),P1.pSC.U(2,1:end),'b-','LineWidth',sizeLineReal);
plot(time(1:end),P2.pSC.Ud(2,1:end),'g--','LineWidth',sizeLineDesired2);
plot(time(1:end),P2.pSC.U(2,1:end),'k-','LineWidth',sizeLineReal2);
legend({'$\omega _{1d}$','$\omega_{1r}$','$\omega _{2d}$','$\omega_{2r}$'},'FontSize',sizeLegend,'Interpreter','latex');
grid on;
% title('Velocidade Angular','fontSize',st);
xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Velocidade angular [rad/s]','FontSize',sizeLabel,'interpreter','Latex');
% legend({'Robot 1', 'Robot 2'},'Interpreter','latex');
xlim([0 time(end)]);
% ylim([-1 1]);
box on


%% Posições dos robôs
figure;
subplot(211);
plot(time,P1.pPos.Xd(1,:),'r--','LineWidth',sizeLineDesired),hold on;
plot(time,P1.pPos.X(1,:),'b-','LineWidth',sizeLineReal);
plot(time,P2.pPos.Xd(1,:),'g--','LineWidth',sizeLineDesired2);
plot(time,P2.pPos.X(1,:),'k-','LineWidth',sizeLineReal2);

legend({'$x_{1d}$','$x_{1r}$','$x_{2d}$','$x_{2r}$'},'FontSize',sizeLegend,'Interpreter','latex');
grid on;
xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Posi\c c\~ao [m]','FontSize',sizeLabel,'interpreter','Latex');
xlim([0 time(end)]);
% ylim([-1 1]);
box on


% figure;
subplot(212);
plot(time,P1.pPos.Xd(2,:),'r--','LineWidth',sizeLineDesired),hold on;
plot(time,P1.pPos.X(2,:),'b-','LineWidth',sizeLineReal);
plot(time,P2.pPos.Xd(2,:),'g--','LineWidth',sizeLineDesired2);
plot(time,P2.pPos.X(2,:),'k-','LineWidth',sizeLineReal2);

legend({'$y_{1d}$','$y_{1r}$','$y_{2d}$','$y_{2r}$'},'FontSize',sizeLegend,'Interpreter','latex');
grid on;
% title('Velocidade Angular','fontSize',st);
xlabel('Tempo [s]','FontSize',sizeLabel,'interpreter','Latex');
ylabel('Posi\c c\~ao [m]','FontSize',sizeLabel,'interpreter','Latex');
% legend({'Robot 1', 'Robot 2'},'Interpreter','latex');
xlim([0 time(end)]);
% ylim([-1 1]);
box on

