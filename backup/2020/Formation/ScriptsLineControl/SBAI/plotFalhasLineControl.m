%% Comandos Iniciais

clear all; clc;
%  close all;
%% Carregando os dados

% Formato em que os dados foram salvos

% %   1 --- 12         13 --- 24        25 --- 26            27 --- 28 
% %   P.pPos.Xd'       P.pPos.X'        P.pSC.Ud(1:2)'       P.pSC.U(1:2)'
% %             
% %   29 --- 40        41 --- 52        53 -- 56             57 -- 60    
% %   A.pPos.Xd'       A.pPos.X'        A.pSC.Ud'            A.pSC.U'
% %             
% %   61 --- 66        67 --- 72        73 -- 78             79
% %   LF.pPos.Qd'      LF.pPos.Qtil'    LF.pPos.Xd'          toc(t)

sl = 1;   %'default';    % largura da linha
st = 12;  % tamanho da fonte
sm = 2;   % tamanho dos símbolos

% filename = 'FalhaPosicaoPioneer_1.mat';

filename = 'FalhaFormaPioneer_1.mat';

% filename = 'FalhaFormaDrone_2.mat';

% filename = 'FalhaPosicaoDrone_3.mat';

% filename = 'FalhaNormalDrone_2.mat';

myVars = {'data'};
Falha.Posicao.P = load(filename,myVars{:});
Falha.Posicao.P = Falha.Posicao.P.data;

%% Plotar dados da mesma estratégia juntos (Forma, Posição ou Normal)
% A partir da comparação entre os resultados de cada tipo, escolher o
% melhor para em seguida comparar cada uma das estratégias de controle

% POSICAO - FALHA NO DRONE

% Tempo da simulação
Time.Posicao.P  = Falha.Posicao.P(:,end);            % tempo (s)

% Pioneer data
PXd.Posicao.P   = Falha.Posicao.P(:,(1:12))';        % desired pose
PX.Posicao.P    = Falha.Posicao.P(:,12+(1:12))';     % real pose
PUd.Posicao.P   = Falha.Posicao.P(:,24+(1:2))';      % control signal
PU.Posicao.P    = Falha.Posicao.P(:,26+(1:2))';      % velocidades do robô
PXtil.Posicao.P = PXd.Posicao.P - PX.Posicao.P;      % erro de postura

% Drone data
AXd.Posicao.P   = Falha.Posicao.P(:,28+(1:12))';     % desired pose
AX.Posicao.P    = Falha.Posicao.P(:,40+(1:12))';     % real pose
AUd.Posicao.P   = Falha.Posicao.P(:,52+(1:4))';      % control signal
AU.Posicao.P    = Falha.Posicao.P(:,56+(1:4))';      % velocidades do robô
AXtil.Posicao.P = AXd.Posicao.P - AX.Posicao.P;      % erro de postura

% Formation data
Qd.Posicao.P    = Falha.Posicao.P(:,60+(1:6))';      % desired formation
Qtil.Posicao.P  = Falha.Posicao.P(:,66+(1:6))';      % formation error

% Erro de Posição
figure;
subplot(211);
plot(Time.Posicao.P ,Qtil.Posicao.P(1,:),'-k','LineWidth',sl);
ylabel('\textit{\~x}$_F$ [m]','interpreter','Latex','FontSize',st);
xlim([0 120]);
ylim([-1.5 1.5]);
yticks([ -1.5 -0.75  0 0.75 1.5]);
grid on;

subplot(212);
plot(Time.Posicao.P ,Qtil.Posicao.P(2,:),'-r','LineWidth',sl);
xlabel('Time [s]','interpreter','Latex','FontSize',st);
ylabel('\textit{\~y}$_F$ [m]','interpreter','Latex','FontSize',st);
xlim([0 120]);
ylim([-1.5 1.5]);
yticks([ -1.5 -0.75  0 0.75 1.5]);
grid on;

% Erro de Forma

figure;
subplot(311);
plot(Time.Posicao.P ,Qtil.Posicao.P(4,:),'-k','LineWidth',sl);
ylabel('$$\tilde{\rho}$$ [m]','interpreter','Latex','FontSize',st);
xlim([0 120]);
ylim([-1.5 1.5]);
yticks([ -1.5 -0.75  0 0.75 1.5]);
grid on;

subplot(312);
plot(Time.Posicao.P ,180/pi*Qtil.Posicao.P(5,:),'-r','LineWidth',sl);
ylabel('$$\tilde{\alpha}$$ [degrees]','interpreter','Latex','FontSize',st);
xlim([0 120]);
ylim([-60 60]);
yticks([ -40 -20  0 20 40 ]);
grid on;

subplot(313);
plot(Time.Posicao.P ,180/pi*Qtil.Posicao.P(6,:),'-b','LineWidth',sl);
xlabel('Time [s]','interpreter','Latex','FontSize',st);
ylabel('$$\tilde{\beta}$$ [degrees]','interpreter','Latex','FontSize',st);
xlim([0 120]);
ylim([-60 60]);
yticks([ -40 -20  0 20 40 ]);
grid on;

%% FALHAS NO DRONE PARA ---->> NP FP PP

clear all; clc;

filename = 'FalhaNormalDrone_2.mat';
myVars = {'data'};
Falha.Normal.D = load(filename,myVars{:});
Falha.Normal.D = Falha.Normal.D.data;

filename = 'FalhaFormaDrone_2.mat';
myVars = {'data'};
Falha.Forma.D = load(filename,myVars{:});
Falha.Forma.D = Falha.Forma.D.data;

filename = 'FalhaPosicaoDrone_3.mat';
myVars = {'data'};
Falha.Posicao.D = load(filename,myVars{:});
Falha.Posicao.D = Falha.Posicao.D.data;

% -----------------------------   NORMAL   ------------------------------- %

% Tempo da simulação
Time.Normal.D  = Falha.Normal.D(:,end);            % tempo (s)

% Pioneer data
PXd.Normal.D   = Falha.Normal.D(:,(1:12))';        % desired pose
PX.Normal.D    = Falha.Normal.D(:,12+(1:12))';     % real pose
PUd.Normal.D   = Falha.Normal.D(:,24+(1:2))';      % control signal
PU.Normal.D    = Falha.Normal.D(:,26+(1:2))';      % velocidades do robô
PXtil.Normal.D = PXd.Normal.D - PX.Normal.D;      % erro de postura

% Drone data
AXd.Normal.D   = Falha.Normal.D(:,28+(1:12))';     % desired pose
AX.Normal.D    = Falha.Normal.D(:,40+(1:12))';     % real pose
AUd.Normal.D   = Falha.Normal.D(:,52+(1:4))';      % control signal
AU.Normal.D    = Falha.Normal.D(:,56+(1:4))';      % velocidades do robô
AXtil.Normal.D = AXd.Normal.D - AX.Normal.D;      % erro de postura

% Formation data
Qd.Normal.D    = Falha.Normal.D(:,60+(1:6))';      % desired formation
Qtil.Normal.D  = Falha.Normal.D(:,66+(1:6))';      % formation error

% -----------------------------   FORMA   ------------------------------- %
Time.Forma.D  = Falha.Forma.D  (:,end);            % tempo (s)

% Pioneer data
PXd.Forma.D   = Falha.Forma.D(:,(1:12))';        % desired pose
PX.Forma.D    = Falha.Forma.D(:,12+(1:12))';     % real pose
PUd.Forma.D   = Falha.Forma.D(:,24+(1:2))';      % control signal
PU.Forma.D    = Falha.Forma.D(:,26+(1:2))';      % velocidades do robô
PXtil.Forma.D = PXd.Forma.D - PX.Forma.D;      % erro de postura

% Drone data
AXd.Forma.D   = Falha.Forma.D(:,28+(1:12))';     % desired pose
AX.Forma.D    = Falha.Forma.D(:,40+(1:12))';     % real pose
AUd.Forma.D   = Falha.Forma.D(:,52+(1:4))';      % control signal
AU.Forma.D    = Falha.Forma.D(:,56+(1:4))';      % velocidades do robô
AXtil.Forma.D = AXd.Forma.D - AX.Forma.D;      % erro de postura

% Formation data
Qd.Forma.D    = Falha.Forma.D(:,60+(1:6))';      % desired formation
Qtil.Forma.D  = Falha.Forma.D(:,66+(1:6))';      % formation error

% ----------------------------   POSICAO   ------------------------------ %
Time.Posicao.D  = Falha.Posicao.D(:,end);            % tempo (s)

% Pioneer data
PXd.Posicao.D   = Falha.Posicao.D(:,(1:12))';        % desired pose
PX.Posicao.D    = Falha.Posicao.D(:,12+(1:12))';     % real pose
PUd.Posicao.D   = Falha.Posicao.D(:,24+(1:2))';      % control signal
PU.Posicao.D    = Falha.Posicao.D(:,26+(1:2))';      % velocidades do robô
PXtil.Posicao.D = PXd.Posicao.D - PX.Posicao.D;      % erro de postura

% Drone data
AXd.Posicao.D   = Falha.Posicao.D(:,28+(1:12))';     % desired pose
AX.Posicao.D    = Falha.Posicao.D(:,40+(1:12))';     % real pose
AUd.Posicao.D   = Falha.Posicao.D(:,52+(1:4))';      % control signal
AU.Posicao.D    = Falha.Posicao.D(:,56+(1:4))';      % velocidades do robô
AXtil.Posicao.D = AXd.Posicao.D - AX.Posicao.D;      % erro de postura

% Formation data
Qd.Posicao.D    = Falha.Posicao.D(:,60+(1:6))';      % desired formation
Qtil.Posicao.D  = Falha.Posicao.D(:,66+(1:6))';      % formation error

%% PLOTAGEM

sl = 1;   %'default';    % largura da linha
st = 12;  % tamanho da fonte
sm = 2;   % tamanho dos símbolos

% Erro de Posição
figure;
subplot(211);
plot(Time.Normal.D ,Qtil.Normal.D(1,:),'-k','LineWidth',sl);
hold on;
plot(Time.Forma.D ,Qtil.Forma.D(1,:),'-or','MarkerIndices',...
    1:40:length(Time.Forma.D),'MarkerSize',sm,'LineWidth',sl)
plot(Time.Posicao.D ,Qtil.Posicao.D(1,:),'-.b','LineWidth',sl);
ylabel('\textit{\~x}$_F$ [m]','interpreter','Latex','FontSize',st);
xlim([0 120]);
ylim([-2 2]);
yticks([ -2 -1 0 1 2]);
lgX = legend('$NP$','$FP$','$PP$');
lgX.FontSize = 10;
lgX.Location = 'northoutside';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(212);
plot(Time.Normal.D ,Qtil.Normal.D(2,:),'-k','LineWidth',sl);
hold on;
plot(Time.Forma.D ,Qtil.Forma.D(2,:),'-or','MarkerIndices',...
    1:40:length(Time.Forma.D),'MarkerSize',sm,'LineWidth',sl)
plot(Time.Posicao.D ,Qtil.Posicao.D(2,:),'-.b','LineWidth',sl);
xlabel('Time [s]','interpreter','Latex','FontSize',st);
ylabel('\textit{\~y}$_F$ [m]','interpreter','Latex','FontSize',st);
xlim([0 120]);
ylim([-2 2]);
yticks([ -2 -1 0 1 2]);
grid on;

% Erro de Forma

figure;
subplot(311);
plot(Time.Normal.D ,Qtil.Normal.D(4,:),'-k','LineWidth',sl);
hold on;
plot(Time.Forma.D ,Qtil.Forma.D(4,:),'-or','MarkerIndices',...
    1:40:length(Time.Forma.D),'MarkerSize',sm,'LineWidth',sl)
plot(Time.Posicao.D ,Qtil.Posicao.D(4,:),'-.b','LineWidth',sl)

ylabel('$$\tilde{\rho}$$ [m]','interpreter','Latex','FontSize',st);
xlim([0 120]);
ylim([-2 2]);
yticks([ -2 -1 0 1 2]);
lgX = legend('$NP$','$FP$','$PP$');
lgX.FontSize = 10;
lgX.Location = 'northoutside';
lgX.Orientation = 'horizontal';
set(lgX,'Interpreter','latex');
grid on;

subplot(312);
plot(Time.Normal.D ,180/pi*Qtil.Normal.D(5,:),'-k','LineWidth',sl);
hold on;
plot(Time.Forma.D ,180/pi*Qtil.Forma.D(5,:),'-or','MarkerIndices',...
    1:40:length(Time.Forma.D),'MarkerSize',sm,'LineWidth',sl);
plot(Time.Posicao.D ,180/pi*Qtil.Posicao.D(5,:),'-.b','LineWidth',sl);
ylabel('$$\tilde{\alpha}$$ [degrees]','interpreter','Latex','FontSize',st);
xlim([0 120]);
ylim([-100 100]);
yticks([ -100 -50  0 50 100 ]);
grid on;

subplot(313);
plot(Time.Normal.D ,180/pi*Qtil.Normal.D(6,:),'-k','LineWidth',sl);
hold on;
plot(Time.Forma.D ,180/pi*Qtil.Forma.D(6,:),'-or','MarkerIndices',...
    1:40:length(Time.Forma.D),'MarkerSize',sm,'LineWidth',sl);
plot(Time.Posicao.D ,180/pi*Qtil.Posicao.D(6,:),'-.b','LineWidth',sl);
xlabel('Time [s]','interpreter','Latex','FontSize',st);
ylabel('$$\tilde{\beta}$$ [degrees]','interpreter','Latex','FontSize',st);
xlim([0 120]);
ylim([-100 100]);
yticks([ -100 -50  0 50 100 ]);
grid on;

