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
    setenv('ROS_IP','192.168.0.104')
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
    RI.rConnect('192.168.0.101');
    
    B = Bebop(1,'B');
    
    J = JoyControl;
       
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



% Ganhos
% v = xd2p + ksp*tanh(kp*Xtil) + ksd*tanh(kd*Xtilponto);
ksp = [1 0 0 0;
       0 1 0 0;
       0 0 3 0;
       0 0 0 1];
ksd = [1.5 0 0 0;
       0 1.5 0 0;
       0 0 2 0;
       0 0 0 2];

kp = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];

kd = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];

k1 = 0.8417;
k2 = 0.18227;
k3 = 0.8354;
k4 = 0.17095;
k5 = 3.966; %4.2492;
k6 = 4.001; %4.3235;
k7 = 9.8524;
k8 = 4.7295;

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
time_pos = 80;
tmax = 120; % Tempo Simulacao em segundos

% Tempo de Amostragem
To = 1/5;

%t  = tic; % Tempo de simulacao
tc = tic; % Tempo de controle
tp = tic; % Tempo de plotagem e exibiï¿½ï¿½o
nLandMsg = 3;    % Numero de mensagens enviadas para pouso do drone
dLandMsg = 1; % Atraso de tempo entre messagens de pouso

%% Decolagem do Drone

% Assegura a decolagem vertical
% Os valores de comando na decolagem sejam iguais a zero
B.pSC.Ud(1) = 0; % Esquerda/Direita [-1,1] (-) Move Drone para Esquerda
B.pSC.Ud(2) = 0; % Frente/Tras [-1,1] (-) Avanï¿½a - Move frente para baixo
B.pSC.Ud(3) = 0; % Velocidade Vertical [-1,1] (+) Eleva o drone
B.pSC.Ud(4) = 0; %

% Get Bebop Odometry
B.rGetSensorDataLocal;

% Inicializa o tempo de simulação
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
xdIndex = 1;
xdPos(:,1) = [5 0 10 0]';
% xdPos(:,1) = [0 0 1.5 0]';
xdPos(:,2) = [5 5 10 pi/2]';
xdPos(:,3) = [0 5 10 pi]';
xdPos(:,4) = [0 0 10 3*pi/2]';
xdPos(:,5) = [0 0 10 0]';
% xdPos(:,5) = [-0.5 -0.5 1 0]';
% xdPos(:,6) = [0 0 1 0]';
% xdPos(:,7) = [0 0 1 0]';
% xdPos(:,8) = [0 0 1 0]';

% Get First Pos
xd = xdPos(:,xdIndex);

% Usar para Trajetoria
xdold = xd;
xdp = (xd-xdold)/To;
xdpold = xdp;
xd2p = (xdp-xdpold)/To;
xd2pold = xd2p;


disp(" ");
disp("Take off Command....")
disp(" ");

% Envia Comando de Decolagem para o Drone e aguarda estabilizaï¿½ï¿½o de
% altitude padrao (8s)
B.rTakeOff;
pause(3);
T = 60;
w = 2*pi/T;

t  = tic;
tsw = tic;

disp(" ");
disp("Estabilization completed .... Turn ON Controller...")
disp(" ");


try
    % while dist > tolerance && toc(t) < tmax
    while toc(t) < tmax
        if toc(tc) > To
            tc = tic;
            
            % Switch desired position
%             if toc(tsw) > time_pos
%                 tsw = tic;
%                 xdIndex = xdIndex + 1;
%                 xd = xdPos(:,xdIndex);
%             end
%             rb = OPT.RigidBody;
%             B.pFlag.isTracked = rb.isTracked;
            B.rGetSensorDataLocal;
            xd = [5*cos(w*toc(t)) 5*sin(2*w*toc(t)) 8 0]';
            xdp = [-5*w*sin(w*toc(t)) 10*w*cos(2*w*toc(t)) 0 0]';
            xd2p = [-5*w.^2*cos(w*toc(t)) -20*w.^2*sin(2*w*toc(t)) 0 0]';
             

%             xdp = 0;%(xd-xdold)/To;
%             xd2p = 0;%(xdp-xdpold)/To;
            
            % Controlador Dinamico Bebop 2
            B.pPos.X(12) = (B.pPos.X(6) - pPosAnt)/To;
            pPosAnt = B.pPos.X(6);
            
            X = [B.pPos.X(1:3); B.pPos.X(6)];
            Xponto_vel = [B.pPos.X(7:9); B.pPos.X(12)];
            
%             F = [cos(X(4)) -sin(X(4))  0  0;
%                 sin(X(4)) cos(X(4))   0  0;
%                 0           0        1  0;
%                 0           0        0 1];
            
%             Xponto = F*Xponto_vel;
            Xponto = Xponto_vel;
            if X(4) < 0
                X(4) = X(4) + 2*pi;
            end
            
            Xtil = xd - X;
            
            %Compensacao Psi
            
            if abs(Xtil(4)) > pi
                if Xtil(4) > pi
                    Xtil(4) = Xtil(4) - 2*pi;
                end
                
                if Xtil(4) < -pi
                    Xtil(4) = Xtil(4) + 2*pi;
                end
            end
            
            Xtilponto = xdp - Xponto;
            
            v = xd2p + ksp*tanh(kp*Xtil) + ksd*tanh(kd*Xtilponto);
            
            
            f1 = [k1*cos(X(4)) -k3*sin(X(4))  0  0;
                k1*sin(X(4)) k3*cos(X(4))  0  0;
                0  0 k5  0;
                0  0  0 k7];
            
            f2 = [k2*cos(X(4)) -k4*sin(X(4))  0  0;
                k2*sin(X(4)) k4*cos(X(4))  0  0;
                0  0 k6  0;
                0  0  0 k8];
            
            U = inv(f1)*(v + f2*Xponto_vel);
            
            dist = norm(xd(1:3)-X(1:3));
            
            % Comandos enviados ao Bebop 2
            B.pSC.Ud(1) = U(1); % v{k}(4)*ganho; % Frente/Tras [-1,1] (+) Avanï¿½a, Move frente para baixo
            B.pSC.Ud(2) = U(2); % -v{k}(5)*ganho; % Esquerda/Direita [-1,1] (+) Move Drone para Esquerda
            B.pSC.Ud(3) = U(3); % -v{k}(6)*ganho; % Velocidade Vertical [-1,1] (+) Eleva o drone
            B.pSC.Ud(4) = 0; % Não Rotaciona
            B.pSC.Ud(5) = 0; % Não Rotaciona
            B.pSC.Ud(6) = U(4); % Angulo do drone [-1,1] (+) rotaciona para esquerda em torno do Eixo Z
            
            comandos = [comandos; B.pSC.Ud(1) B.pSC.Ud(2) B.pSC.Ud(3) B.pSC.Ud(6)];
            poses = [poses; B.pPos.X(1) B.pPos.X(2) B.pPos.X(3) B.pPos.X(6)];
            velocidades = [velocidades; B.pPos.X(7) B.pPos.X(8) B.pPos.X(9) B.pPos.X(12)];
            dists = [dists; dist];
            Bebop = [Bebop;B];
            xds = [xds xd];
            xdps = [xdps xdp];
            
            tempo = [tempo toc(t)];
                    
%             B.pSC.Ud = [0 0 0 0 0 0]';
            
            % Joystick Command Priority
            B = J.mControl(B);                    % joystick command (priority)
            B.pSC.Ud
            % Envia Comando para o Drone
            B.rCommand;
            
            xdold = xd;
            xdpold = xdp;
            xd2pold = xd2p;
            
            drawnow;
        end
        
        % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
        if btnEmergencia ~= 0 || B.pFlag.EmergencyStop ~= 0 || B.pFlag.isTracked ~= 1
            disp('Bebop Landing through Emergency Command ');
            B.rCmdStop;
            B.rLand;
            break;
        end    
    end
catch
    disp('Bebop Landing through Try/Catch Loop Command');
    B.rCmdStop;      
end

% Send 3 times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    disp("End Land Command");
    B.rCmdStop;    
    B.rLand
    pause(dLandMsg);
end


% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");

%%
filename = ['bebop_Externo_ICUAS_Pos' datestr(now,30) '.mat'];
save(filename,'comandos','poses','velocidades','dists','Bebop','xds','xdps')


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

disp("Figures Printed...");