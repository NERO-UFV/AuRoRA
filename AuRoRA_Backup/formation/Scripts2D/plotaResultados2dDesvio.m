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
P1.pSC.Ur    = data(:,24+(1:2))';     % sinal de controle
P1.pSC.U     = data(:,26+(1:2))';     % velocidades do robô
P1.pPos.Xtil = P1.pPos.Xd - P1.pPos.X; % erro de postura

% Robô B
P2.pPos.Xd   = data(:,28+(1:12))';
P2.pPos.X    = data(:,40+(1:12))';
P2.pSC.Ur    = data(:,52+(1:2))';
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
Qd0 = [8.01  0  .89  pi/2]';
% Após 25% do percurso:
Qd1 = [8.01  0  .89   0]';
% Após 65% do percurso:
Qd2 = [8.01  0  .89  -pi/2]';



% Vetor de posição dos robôs: LF.pPos.X = [x1 y1 z1 rho1 theta1 psi1 x2 y2 z2 rho2 theta2 psi2]
% Vetor da formação: LF.pPos.Q = [xf yf rhof alfaf]
for k=1:length(time)
    LF.pPos.X = [P1.pPos.X(1:6,k) P2.pPos.X(1:6,k)];
    LF.mDirTrans;
    Q(:,k) = LF.pPos.Q;
    
        if LF.pPos.Q(1)>=LF.pPos.Qd(1)*.65  % obstáculo a X% do caminho      
           
            Qd(:,k) = Qd2;
      
        elseif LF.pPos.Q(1)>=LF.pPos.Qd(1)*.25 % 
        
            Qd(:,k) = Qd1;  
        else
            
            Qd(:,k) = Qd0;
        end
    
    Qtil(:,k) = Qd(:,k) - Q(:,k);
end

%% PLOTA RESULTADOS
sizeLine= 1;%'default';    % largura da linha
st = 14;  % tamanho da fonte
ss = 2;   % tamanho dos símbolos

%% Posição dos robôs
figure(1);
axis equal
hold on, grid on;

% percurso realizado
p1 = plot(P1.pPos.X(1,:),P1.pPos.X(2,:),'r-','LineWidth',sizeLine);
p2 = plot(P2.pPos.X(1,:),P2.pPos.X(2,:),'b-','LineWidth',sizeLine);
% title('Posição dos Robôs','fontSize',lt);
xlabel('$x$ [m]','interpreter','Latex'),ylabel('$y$ [m]','Interpreter','latex');

% % % Percurso desejado
% p1 = plot(A.pPos.Xd(:,1),A.pPos.Xd(:,2),'r-','LineWidth',sl);
% p2 = plot(B.pPos.Xd(:,1),B.pPos.Xd(:,2),'b-','LineWidth',sl);
%

% % posições iniciais
% plot(A.pPos.X(1,1),A.pPos.X(1,2) ,'ro','LineWidth',ss); % início
% plot(B.pPos.X(1,1),B.pPos.X(1,2) ,'bo','LineWidth',ss); % inicio
% plot(A.pPos.X(end,1),A.pPos.X(end,2) ,'rx','LineWidth',ss); % chegada
% plot(B.pPos.X(end,1),B.pPos.X(end,2),'bx','LineWidth',ss);  % chegada

% plota robos e linhas de formação
for k = 1:80:length(time)
    
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

lg1 = legend([p1 p2 pl],{'Robot 1', 'Robot 2','Formation'});

lg1.FontSize = 11;
lg1.Location = 'SouthEast';
set(lg1,'Interpreter','latex');
% legend('boxoff')
box on
% % Salva a imagem
% saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\posicao.png');

%% Velocidades lineares
figure;
hold on, grid on;
plot(time,P1.pSC.U(1,:),'r','LineWidth',sizeLine);
plot(time,P2.pSC.U(1,:),'b','LineWidth',sizeLine);
% title('Velocidade Linear','fontSize',st);
xlabel('Time [s]','interpreter','Latex'),ylabel('Velocity [m/s]','interpreter','Latex');
legend({'Robot 1', 'Robot 2'},'Interpreter','latex');
xlim([0 time(end)]);
% % Salva a imagem
% saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\velLinear.png');


% % Sinal de controle - velocidade linear
% figure;
% hold on, grid on;
% plot(time,A.pSC.Ur(:,1),'LineWidth',sl);
% plot(time,B.pSC.Ur(:,1),'LineWidth',sl);
% title('Sinal de controle (Velocidade Linear)','fontSize',st);
% xlabel('$Tempo$ [s]','interpreter','Latex'),ylabel('$Velocidade$ [m/s]','interpreter','Latex');
% legend('Robô 1', 'Robô 2');


% Velocidades Angulares
figure;
hold on, grid on;
plot(time,P1.pSC.U(2,:),'r','LineWidth',sizeLine);
plot(time,P2.pSC.U(2,:),'b','LineWidth',sizeLine);
% title('Velocidade Angular','fontSize',st);
xlabel('Time [s]','interpreter','Latex'),ylabel('Velocity [rad/s]','interpreter','Latex');
legend({'Robot 1', 'Robot 2'},'Interpreter','latex');
xlim([0 time(end)]);

% % Salva a imagem
% saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\velAngular.png');


% % Sinal de controle - velocidade angular
% figure;
% hold on, grid on;
% plot(time,A.pSC.Ur(:,2),'LineWidth',sl);
% plot(time,B.pSC.Ur(:,2),'LineWidth',sl);
% title('Sinal de controle (Velocidade Angular)','fontSize',st);
% xlabel('$Tempo$ [s]','interpreter','Latex'),ylabel('$Velocidade$ [rad/s]','interpreter','Latex');
% legend('Robô 1', 'Robô 2');

%% Erros
% % Erro de posição
% figure;
% hold on, grid on;
% plot(time,A.pPos.Xtil(:,[1 2]),'LineWidth',sl);
% plot(time,B.pPos.Xtil(:,[1 2]),'LineWidth',sl);
% % title('Erro de posição','fontSize',st);
% xlabel('$Time$ [s]','interpreter','Latex'),ylabel('Error [m]','interpreter','Latex');
% xlim([0 time(end)]);
%
% lg2 = legend('$x_{robot 1}$','$y_{robot 1}$', '$x_{robot 2}$','$y_{robot 2}$');
% lg2.FontSize = 12;
% lg2.Location = 'SouthEast';
% set(lg2,'Interpreter','latex');
%
% % % Salva a imagem
% % saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\ErroPosicao.png');


% Erro da formação
figure;
hold on, grid on;
plot(time,Qtil(1:3,:),'LineWidth',sizeLine);
% title('Erro de posição','fontSize',st);
xlabel('$Time$ [s]','interpreter','Latex'),ylabel('Error','interpreter','Latex');
xlim([0 time(end)]);

lg3 = legend('$x_f [m]$','$y_f [m]$', '$\rho_f [m]$','$\alpha_f [rad]$');
lg3.FontSize = 12;
lg3.Location = 'NorthEast';
set(lg3,'Interpreter','latex');

% % Salva a imagem
% saveas(gcf,'C:\Users\mfrab\Google Drive\Mestrado\Dissertação\graficos\Erroformacao.png');


