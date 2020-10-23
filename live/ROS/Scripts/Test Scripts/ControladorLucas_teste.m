%% Tarefa 1

% Limpa as varaveis
clear all;
close all;
warning off;
clc;

% Carrega o diretï¿½rio corrente e subdiretï¿½rios

%% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%%

btnEmergencia = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Drone  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
try
    fclose(instrfindall);
end


%% Load Class
try
    % Load Classes
    RI = RosInterface;
    %     setenv('ROS_MASTER_URI','http://192.168.0.106:11311')
    %     setenv('ROS_IP','192.168.0.101')
    %     RI.rConnect('172.20.24.221');
    RI.rConnect('192.168.0.103');
    B = Bebop(1);
    
    disp('################### Load Class Success #######################');
    
catch ME
    disp(' ');
    disp(' ################### Load Class Issues #######################');
    disp(' ');
    disp(' ');
    disp(ME);
    
    RI.rDisconnect;
    rosshutdown;
    return;
end
%%

ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Drone %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Condicoes iniciais
x(1) = 0;
y(1) = 0;
z(1) = 1;
phi(1) = 0;

% Ganhos
kp = [0.5 0 0 0;
      0 0.4 0 0;
      0 0 0.2 0;
      0 0 0 1];

kd = [1 0 0 0;
      0 1 0 0;
      0 0 1 0;
      0 0 0 1];

ksp = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];

ksd = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
  
k1 = 4.3321;
k2 = 0.2333;
k3 = 1.1161;
k4 = 0.37855;
k5 = 4.2492;
k6 = 4.3235;
k7 = 10.4174;
k8 = 5.9794;

f1 = [k1  0  0  0;
    0 k3  0  0;
    0  0 k5  0;
    0  0  0 k7];

f2 = [k2  0  0  0;
    0 k4  0  0;
    0  0 k6  0;
    0  0  0 k8];

%%%%%%%%%%%%%%%%%%%%%% Parametros do Teste %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Configuracao do teste
tmax = 1200; % Tempo Simulacao em segundos

% Tempo de Amostragem
To = 1/5;

%t  = tic; % Tempo de simulacao
tc = tic; % Tempo de controle
tp = tic; % Tempo de plotagem e exibiï¿½ï¿½o
nLandMsg = 5;    % Numero de mensagens enviadas para pouso do drone
dLandMsg = 1; % Atraso de tempo entre messagens de pouso

%% Decolagem do Drone

% Assegura a decolagem vertical
% Os valores de comando na decolagem sejam iguais a zero
B.pSC.Ud(1) = 0; % Esquerda/Direita [-1,1] (-) Move Drone para Esquerda
B.pSC.Ud(2) = 0; % Frente/Tras [-1,1] (-) Avanï¿½a - Move frente para baixo
B.pSC.Ud(3) = 0; % Velocidade Vertical [-1,1] (+) Eleva o drone
B.pSC.Ud(4) = 0; %

B.rGetSensorDataLocal;

% Envia Comando de Decolagem para o Drone e aguarda estabilizaï¿½ï¿½o de
% altitude padrao (8s)
B.rTakeOff;
pause(4);

disp(" ");
disp("Final da Estabilizaï¿½ao .... Controlador ON...")
disp(" ");

% Inicializa o tempo de simulação
t  = tic;
comandos = [];
poses = [];
velocidades = [];
tempo = [];
Bebop = [];
dists=[];
xds=[];
xdps=[];

pPosAnt = B.pPos.X(6);
xd(4) = B.pPos.X(6);

tolerance = 0.1;
dist = 1000;
% mult = 2;
% mult2 = 0.1;
% xd = [mult*cos(mult2*toc(t)) mult*sin(mult2*toc(t)) 1 B.pPos.X(6)]';
xd = [5 3 4 0]';
xdold = xd;
xdp = (xd-xdold)/To;
xdpold = xdp;
xd2p = (xdp-xdpold)/To;
xd2pold = xd2p;

while dist > tolerance
% while toc(t) < tmax
    if toc(tc) > To
        tc = tic;
        
        B.rGetSensorDataLocal;
        
        xd = [5 3 4 0]';
        xdp = (xd-xdold)/To;
        xd2p = (xdp-xdpold)/To;
        
        % Controlador Dinamico Bebop 2
        B.pPos.X(12) = (B.pPos.X(6) - pPosAnt)/To;
        pPosAnt = B.pPos.X(6);
        
        X = [B.pPos.X(1:3); B.pPos.X(6)];
        Xponto_vel = [B.pPos.X(7:9); B.pPos.X(12)];
        
        F = [cos(X(4)) -sin(X(4))  0  0;
            sin(X(4)) cos(X(4))  0  0;
            0  0 1  0;
            0  0  0 1];
        
        Xponto = F*Xponto_vel;
        
        Xtil = xd - X;
        Xtilponto = xdp - Xponto;
        
        v = xd2p + kp*tanh(ksp*Xtil) + kd*tanh(ksd*Xtilponto);
        
        f1 = [k1*cos(X(4)) -k3*sin(X(4))  0  0;
            k1*sin(X(4)) k3*cos(X(4))  0  0;
            0  0 k5  0;
            0  0  0 k7];
        
        f2 = [k2*cos(X(4)) k4*sin(X(4))  0  0;
            k2*sin(X(4)) k4*cos(X(4))  0  0;
            0  0 k6  0;
            0  0  0 k8];
        
        U = inv(f1)*(v + f2*Xponto);
        
        dist = norm(xd(1:3)-X(1:3));
        
        % Comandos enviados ao Bebop 2
        B.pSC.Ud(1) = U(1); % v{k}(4)*ganho; % Frente/Tras [-1,1] (+) Avanï¿½a, Move frente para baixo
        B.pSC.Ud(2) = U(2); % -v{k}(5)*ganho; % Esquerda/Direita [-1,1] (+) Move Drone para Esquerda
        B.pSC.Ud(3) = U(3); % -v{k}(6)*ganho; % Velocidade Vertical [-1,1] (+) Eleva o drone
        B.pSC.Ud(4) = 0; % Não Rotaciona
        B.pSC.Ud(5) = 0; % Não Rotaciona
        B.pSC.Ud(6) = 0; % Angulo do drone [-1,1] (+) rotaciona para esquerda em torno do Eixo Z
        
        comandos = [comandos; B.pSC.Ud(1) B.pSC.Ud(2) B.pSC.Ud(3) B.pSC.Ud(6)];
        poses = [poses; B.pPos.X(1) B.pPos.X(2) B.pPos.X(3) B.pPos.X(6)];
        velocidades = [velocidades; B.pPos.X(7) B.pPos.X(8) B.pPos.X(9) B.pPos.X(12)];
        dists = [dists; dist];
        Bebop = [Bebop;B];
        xds = [xds xd];
        xdps = [xdps xdp];
        
        tempo = [tempo toc(t)];
        
        
        % Envia Comando para o Drone
        B.rCommand;
        
        xdold = xd;
        xdpold = xdp;
        xd2pold = xd2p;
        
        drawnow;
        
    end
    
    if btnEmergencia ~= 0
        disp('Bebop Landing');
        B.rLand;
        break;
    end
    
end


% Send 3 times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    B.rLand
    pause(dLandMsg);
end


% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Comando de Pouso ao final do programa");


%%
figure
hold on
plot(tempo,poses(:,1))
plot(tempo,poses(:,2))
plot(tempo,poses(:,3))
plot(tempo,poses(:,4))
title('Posicoes')
legend('Pos X','Pos Y','Pos Z', 'Ori Z');
xlabel('Tempo(s)');
ylabel('Pos');
hold off

figure
hold on
plot(tempo,velocidades(:,1))
plot(tempo,velocidades(:,2))
plot(tempo,velocidades(:,3))
plot(tempo,velocidades(:,4))
title('Velocidades')
xlabel('Tempo(s)');
ylabel('Vel');

legend('Vel X','Vel Y','Vel Z', 'Vel Ang Z');
hold off

figure
hold on
plot(tempo,comandos(:,1))
plot(tempo,comandos(:,2))
plot(tempo,comandos(:,3))
title('Comandos')
xlabel('Tempo(s)');
ylabel('U');

legend('U X','U Y','U Z');
hold off


figure
hold on
plot(tempo,dists)
title('Dist')
hold off
