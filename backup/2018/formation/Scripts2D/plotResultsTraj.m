%% Rotina para plotar gráficos de Formacaolinha2d trajetória

function plotResultsTraj(P1,P2,LF,data)
%% Atribuição de variáveis
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

% Dados da Formação
LF.pPos.Qd    = data(:,56+(1:4));
LF.pPos.Qtil  = data(:,60+(1:4));

% Vetor de posição dos robôs: LF.pPos.X = [x1 y1 z1 rho1 theta1 psi1 x2 y2 z2 rho2 theta2 psi2]
% Vetor da formação: LF.pPos.Q = [xf yf rhof alfaf]
for k = 1:length(time)
    %     if A.pID==1
    LF.pPos.X = [P1.pPos.X(k,1:6) P2.pPos.X(k,1:6)];
    %     elseif A.pID==2
    %     LF.pPos.X = [B.pPos.X(k,1:6) A.pPos.X(k,1:6)];
    %     end
    LF.mDirTrans;
    Q(:,k) = LF.pPos.Q';
end

%% PLOTA RESULTADOS
sl= .5;%'default';    % largura da linha
% st = 14;  % tamanho da fonte
% ss = 2;   % tamanho dos símbolos

%% Posição dos robôs
figure;
axis equal
hold on, grid on;

% percurso realizado
p1 = plot(P1.pPos.X(:,1),P1.pPos.X(:,2),'r-','LineWidth',sl);
p2 = plot(P2.pPos.X(:,1),P2.pPos.X(:,2),'b-','LineWidth',sl);
% title('Posição dos Robôs','fontSize',lt);
xlabel('$x$ [m]','interpreter','Latex'),ylabel('$y$ [m]','Interpreter','latex');

% trajetória da formação desejada
pd  = plot(LF.pPos.Qd(:,1),LF.pPos.Qd(:,2),'k-','LineWidth',2);
% trajetória da formação real
pd2 = plot(Q(1,:),Q(2,:),'c--','LineWidth',2.5);

% plota robos e linhas de formação
for k = 1:170:length(time)
    
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
    pl.LineWidth = 1;
    
    hold on
%     
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
lg1 = legend([p1 p2 pl pd pd2],{'Rob\^{o} 1', 'Rob\^{o} 2','Linha da Forma\c c\~{a}o','Trajet\''{o}ria desejada','Trajet\''{o}ria real'});

lg1.FontSize = 11;
lg1.Location = 'NorthEast';
set(lg1,'Interpreter','latex');
% legend('boxoff')

% % Salva a imagem
% saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\posicao.png');

%% Velocidades lineares
figure;
subplot(211)
plot(time,P1.pSC.Ud(:,1),'LineWidth',sl),hold on;
plot(time,P1.pSC.U(:,1),'LineWidth',sl);
xlabel('Tempo [s]','interpreter','Latex'),ylabel('Velocidade [m/s]','interpreter','Latex');
legend({'$u_{1d}$','$u_{1r}$'},'FontSize',12,'Interpreter','latex');
xlim([0 time(end)]);
grid on;

subplot(212)
plot(time,P2.pSC.Ud(:,1),'LineWidth',sl),hold on;
plot(time,P2.pSC.U(:,1),'LineWidth',sl);
% title('Velocidade Linear','fontSize',st);
xlabel('Tempo [s]','interpreter','Latex'),ylabel('Velocidade [m/s]','interpreter','Latex');
% legend({'Robot 1', 'Robot 2'},'Interpreter','latex');
legend({'$u_{2d}$','$u_{2r}$'},'FontSize',12,'Interpreter','latex');
xlim([0 time(end)]);
grid on;

% comparação entre velocidades lineares dos robos
figure;
pu1 = plot(time(1:end-5),P1.pSC.U(1:end-5,1),'r--','LineWidth',sl);hold on;
pu2 = plot(time(1:end-5),P2.pSC.U(1:end-5,1),'b','LineWidth',sl);
xlabel('Tempo [s]','interpreter','Latex'),ylabel('Velocidade [m/s]','interpreter','Latex');
xlim([0 time(end)]);
% ylim([0 0.6]);
grid on;
lv0 = legend([pu1 pu2],{'Rob\^{o} 1', 'Rob\^{o} 2'});
lv0.FontSize = 12;
% lv.Location = 'SouthEast';
set(lv0,'Interpreter','latex');



% Velocidades Angulares -------------------------------------
figure;
subplot(211);
plot(time,P1.pSC.Ud(:,2),'LineWidth',sl),hold on;
plot(time,P1.pSC.U(:,2),'LineWidth',sl);
legend({'$\omega_{1d}$','$\omega_{1r}$'},'FontSize',12,'Interpreter','latex');
xlabel('Tempo [s]','interpreter','Latex'),ylabel('Velocidade [rad/s]','interpreter','Latex');
xlim([0 time(end)]);
grid on;

subplot(212);
plot(time,P2.pSC.Ud(:,2),'LineWidth',sl),hold on;
plot(time,P2.pSC.U(:,2),'LineWidth',sl);
legend({'$\omega _{2d}$','$\omega_{2r}$'},'FontSize',12,'Interpreter','latex');
grid on;
% title('Velocidade Angular','fontSize',st);
xlabel('Tempo [s]','interpreter','Latex'),ylabel('Velocidade [rad/s]','interpreter','Latex');
% legend({'Robot 1', 'Robot 2'},'Interpreter','latex');
xlim([0 time(end)]);


% comparação entre velocidades angulares dos robos
figure;
pw1 = plot(time(1:end-5),P1.pSC.U(1:end-5,2),'r--','LineWidth',sl);hold on;
pw2 = plot(time(1:end-5),P2.pSC.U(1:end-5,2),'b','LineWidth',sl);
xlabel('Tempo [s]','interpreter','Latex'),ylabel('Velocidade [rad/s]','interpreter','Latex');
lv = legend([pw1 pw2],{'Rob\^{o} 1', 'Rob\^{o} 2'});
lv.FontSize = 12;
% lv.Location = 'SouthEast';
set(lv,'Interpreter','latex');

xlim([0 time(end)]);
% ylim([0 0.6]);
grid on

%% Erros
% Erro da formação - posição
figure;
hold on, grid on;
plot(time(1:end-5),LF.pPos.Qtil(1:end-5,1:3),'LineWidth',sl);
% title('Erro de posição','fontSize',st);
xlabel('Tempo [s]','interpreter','Latex'),ylabel('Erro [m]','interpreter','Latex');
xlim([0 time(end)]);
box on
lg3 = legend('$x_f$','$y_f$', '$\rho_f$');
% lg3 = legend('$x_f [m]$','$y_f [m]$', '$\rho_f [m]$');

lg3.FontSize = 12;
lg3.Location = 'NorthEast';
set(lg3,'Interpreter','latex');

% Angulo
figure;
hold on, grid on;
plot(time(1:end-5),LF.pPos.Qtil(1:end-5,4),'LineWidth',sl);
% title('Erro de posição','fontSize',st);
xlabel('Tempo [s]','interpreter','Latex'),ylabel('Erro [rad]','interpreter','Latex');
xlim([0 time(end)]);
box on
% % Salva a imagem
% saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\Erroformacao.png');


