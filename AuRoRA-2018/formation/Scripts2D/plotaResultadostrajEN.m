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
A = Pioneer3DX;
B = Pioneer3DX;
LF = LineFormation2D('center');

%% Carrega dados
data = load('FL2dtraj_20180630T155638.txt');
% data = load('FL2dtraj_20180629T184601.txt');
% data = load('FL2dtraj_20180704T152730.txt');
% data = load('FL2dtraj_20180629T183816.txt');
%% Atribuição de variáveis
% data = data(1:1000,:);
%Tempo da simulação
time  = data(:,end);                % tempo (s)

% Robô A
A.pPos.Xd   = data(:,(1:12));       % postura desejada
A.pPos.X    = data(:,12+(1:12));    % postura real
A.pSC.Ud    = data(:,24+(1:2));     % sinal de controle
A.pSC.U     = data(:,26+(1:2));     % velocidades do robô
A.pPos.Xtil = A.pPos.Xd - A.pPos.X; % erro de postura

% Robô B
B.pPos.Xd   = data(:,28+(1:12));
B.pPos.X    = data(:,40+(1:12));
B.pSC.Ud    = data(:,52+(1:2));
B.pSC.U     = data(:,54+(1:2));
B.pPos.Xtil = B.pPos.Xd - B.pPos.X;

% Dados da Formação
LF.pPos.Qd    = data(:,56+(1:4));
LF.pPos.Qtil = data(:,60+(1:4));


% Vetor de posição dos robôs: LF.pPos.X = [x1 y1 rho1 theta1 psi1 x2 y2 z2 rho2 theta2 psi2]
% Vetor da formação: LF.pPos.Q = [xf yf rhof alfaf]
for k=1:length(time)
    LF.pPos.X = [A.pPos.X(k,1:6) B.pPos.X(k,1:6)];
    LF.mDirTrans;
    Q(:,k) = LF.pPos.Q';
 
end

%% PLOTA RESULTADOS
sl= 1;%'default';    % largura da linha
st = 14;  % tamanho da fonte
ss = 2;   % tamanho dos símbolos


%% Posição dos robôs
figure;
% axis([-5 5 -5 5]);
axis equal
hold on, grid on;

% percurso realizado
p1 = plot(A.pPos.X(:,1),A.pPos.X(:,2),'r-','LineWidth',sl);
p2 = plot(B.pPos.X(:,1),B.pPos.X(:,2),'b-','LineWidth',sl);
% title('Posição dos Robôs','fontSize',lt);
xlabel('$x$ [m]','interpreter','Latex'),ylabel('$y$ [m]','Interpreter','latex');

% trajetória da formação desejada
pd  = plot(LF.pPos.Qd(:,1),LF.pPos.Qd(:,2),'k-','LineWidth',2);
% trajetória da formação real
pd2 = plot(Q(1,:),Q(2,:),'c--','LineWidth',3);

% plota robos e linhas de formação
for k = 1:150:length(time)
    
    % Obtenção da posição do centro dos robôs
    A.pPos.Xc([1 2 6]) = A.pPos.X(k,[1 2 6])' - ...
        [A.pPar.a*cos(A.pPos.X(k,6)); A.pPar.a*sin(A.pPos.X(k,6)); 0];
    
    B.pPos.Xc([1 2 6]) = B.pPos.X(k,[1 2 6])' - ...
        [B.pPar.a*cos(B.pPos.X(k,6)); B.pPar.a*sin(B.pPos.X(k,6)); 0];
    
    
        % plota trianglinho
        A.mCADplot2D('r');
        B.mCADplot2D('b');
    
    %     % plota pioneer
%     A.mCADplot(1,'r');
%     B.mCADplot(1,'b');
    
    % plot linhas da formação executadas
    x = [A.pPos.X(k,1) B.pPos.X(k,1)];
    y = [A.pPos.X(k,2) B.pPos.X(k,2)];
    
    pl = line(x,y);
    pl.Color = 'g';
    pl.LineStyle = '-';
    pl.LineWidth = 1;
    
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
lg1 = legend([p1 p2 pl pd pd2],{'Robot 1', 'Robot 2','Formation Line','Desired trajectory','Real trajectory'});

lg1.FontSize = 10;
lg1.Location = 'NorthEast';
set(lg1,'Interpreter','latex');
% legend('boxoff')

% % Salva a imagem
% saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\posicao.png');

%% Velocidades lineares
figure;
subplot(211)
plot(time(1:end-5),A.pSC.Ud(1:end-5,1),'r--','LineWidth',sl),hold on;
plot(time(1:end-5),A.pSC.U(1:end-5,1),'b','LineWidth',sl);
legend({'$u_{1des}$','$u_{1real}$'},'Interpreter','latex');
xlabel('$Time$ [s]','interpreter','Latex'),ylabel('$Velocity$ [m/s]','interpreter','Latex');
xlim([0 time(end)]);
grid on;

subplot(212)
plot(time(1:end-5),B.pSC.Ud(1:end-5,1),'r--','LineWidth',sl),hold on;
plot(time(1:end-5),B.pSC.U(1:end-5,1),'b','LineWidth',sl);
% title('Velocidade Linear','fontSize',st);
xlabel('$Time$ [s]','interpreter','Latex'),ylabel('$Velocity$ [m/s]','interpreter','Latex');
% legend({'Robot 1', 'Robot 2'},'Interpreter','latex');
legend({'$u_{2des}$','$u_{2real}$'},'Interpreter','latex');
xlim([0 time(end)]);
grid on
% % Salva a imagem
% saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\velLinear.png');


% % Sinal de controle - velocidade linear
% figure;
% hold on, grid on;
% plot(time,A.pSC.Ud(:,1),'LineWidth',sl);
% plot(time,B.pSC.Ud(:,1),'LineWidth',sl);
% title('Sinal de controle (Velocidade Linear)','fontSize',st);
% xlabel('$Time$ [s]','interpreter','Latex'),ylabel('$Velocity$ [m/s]','interpreter','Latex');
% legend('Robot 1', 'Robot 2');


% Velocidades Angulares -------------------------------------
figure;
subplot(211);
plot(time(1:end-5),A.pSC.Ud(1:end-5,2),'r--','LineWidth',sl),hold on;
plot(time(1:end-5),A.pSC.U(1:end-5,2),'b','LineWidth',sl);
legend({'$\omega_{1des}$','$\omega_{1real}$'},'Interpreter','latex');
xlabel('$Time$ [s]','interpreter','Latex'),ylabel('$Velocity$ [rad/s]','interpreter','Latex');
xlim([0 time(end)]);
grid on;

subplot(212);
plot(time(1:end-5),B.pSC.Ud(1:end-5,2),'r--','LineWidth',sl),hold on;
plot(time(1:end-5),B.pSC.U(1:end-5,2),'b','LineWidth',sl);
legend({'$\omega _{2des}$','$\omega_{2real}$'},'Interpreter','latex');
grid on;
% title('Velocidade Angular','fontSize',st);
xlabel('$Time$ [s]','interpreter','Latex'),ylabel('$Velocity$ [rad/s]','interpreter','Latex');
% legend({'Robot 1', 'Robot 2'},'Interpreter','latex');
xlim([0 time(end)]);

% % Salva a imagem
% saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\velAngular.png');


% % Sinal de controle - velocidade angular
% figure;
% hold on, grid on;

% title('Sinal de controle (Velocidade Angular)','fontSize',st);
% xlabel('$Tempo$ [s]','interpreter','Latex'),ylabel('$Velocidade$ [rad/s]','interpreter','Latex');
% legend('Robô 1', 'Robô 2');

%% Erros
% Erro da formação - posição
figure;
hold on, grid on;
plot(time(1:end-5),LF.pPos.Qtil(1:end-5,1:3),'LineWidth',sl);
% title('Erro de posição','fontSize',st);
xlabel('$Time$ [s]','interpreter','Latex'),ylabel('Error','interpreter','Latex');
xlim([0 time(end)]);

lg3 = legend('$x_f$','$y_f$', '$\rho_f$');
% lg3 = legend('$x_f [m]$','$y_f [m]$', '$\rho_f [m]$');

lg3.FontSize = 12;
lg3.Location = 'NorthEast';
set(lg3,'Interpreter','latex');

% erro - Angulo
figure;
hold on, grid on;
plot(time(1:end-5),LF.pPos.Qtil(1:end-5,4),'LineWidth',sl);
% title('Erro de posição','fontSize',st);
xlabel('Tempo [s]','interpreter','Latex'),ylabel('Erro [rad]','interpreter','Latex');
xlim([0 time(end)]);

